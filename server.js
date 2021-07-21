const express = require('express')
const mongoose = require('mongoose')
const cors = require('cors')
const Inventory = require('./model/inventory.model')
const Queue = require('./model/queue.model')

const path = require('path')
const Pusher = require('pusher')

require('dotenv').config()

const app = express()
const uri = process.env.ATLAS_URI

const connection = mongoose.connection
connection.once('open',() => {
    console.log("MongoDB database connection established successfully")
})

const carSocketId = {}

connection.on('error', console.error.bind(console, 'Connection Error:'))

mongoose.connect(uri,{
    useNewUrlParser:true,
    useCreateIndex:true,
    useUnifiedTopology: true,
    useFindAndModify: false
})

const inventoryWatch = Inventory.watch()
const queueWatch = Queue.watch()

const pusher = new Pusher({
    appId: "1219725",
    key: "2ccb32686bdc0f96f50a",
    secret: "84a8c245f48597b3e0bb",
    cluster: "ap1",
    useTLS: true
})
const channel = 'tasks'

const loginRouter = require('./routes/userLogin')
const inventoryRouter = require('./routes/inventoryLoading')
const sortRouter = require('./routes/sort')
const popupRouter = require('./routes/popup')

const PORT = process.env.PORT || 5000
const httpServer = require('http').createServer(app).listen(PORT)

app.use(cors())
app.use(express.json())


app.use('/login',loginRouter)
app.use('/inventory',inventoryRouter)
app.use('/sort',sortRouter)
app.use('/queue',popupRouter)


if (process.env.NODE_ENV === "production"){
    app.use(express.static(path.join(__dirname,"client", "build")))

    app.get("*", (req, res) => {
        res.sendFile(path.join(__dirname,"client", "build", "index.html"));
    })
}

const io = require('socket.io')(httpServer,{
    cors:{
        origin: [`http://localhost:3000`],
        methods: ["GET", "POST"],
    },
})

async function queueData(carSocketId){
    return await Queue.countDocuments(function(err,count){
        if(!err && count !== 0){
            Queue.findOne().sort({ createdAt: 1 })
            .exec((err, item) => {
                if (err) return (err)
                if(carSocketId !== undefined){
                    console.log("this")
                    io.to(carSocketId).emit('sending-search',{'keyword':item.keyword,'userId':item.userId})
                }
            })
        }
    })
}

io.on("connection",async socket =>{
    // console.log('server connected '+socket.id)
    socket.broadcast.emit("popup",true)


    // const carSocketIdListener = async (delta) =>{
    //     carSocketId[socket.id] = delta
        // console.log('car socket '+carSocketId[socket.id])
        // await queueData(carSocketId[socket.id])
    // }
    // socket.on('car-socket-id',carSocketIdListener)

    // const raspberrySendListener = (delta) =>{
    //     socket.broadcast.emit('receive-raspberry',delta)
    //     // console.log("this "+delta['velocity'])
    // }
    // socket.on('raspberry-send',raspberrySendListener)

    socket.on('begin-search',async delta =>{
        await createQueue(delta).catch(err =>{console.log(err)})
        // console.log("from front-end "+delta['socketId'])
    })

    queueWatch.on('change',async (change)=>{
        if(change.operationType === 'insert' ) {
            // await queueData(carSocketId[socket.id])
            socket.broadcast.emit("popup",true)
        }
    })

    // const sendingResultListener = async (delta) =>{
    //     const content = {
    //         'location':delta['location'],
    //         'photo':delta['photo'],
    //         'userId':delta['userId']
    //     }
    //     deleteFirstItem()
    //     await createInventory(content).catch(err => {console.log(err)})
    //     console.log('server received ',delta['location'])
    // }
    // socket.on('sending-result',sendingResultListener)
    

    // const str = "testing this line"
    // socket.broadcast.emit('sending',str)

    // socket.on('get-user', async userId =>{
        // socket.on('send-changes',delta =>{
        //     // console.log("you see this ",delta)
        //     socket.broadcast.emit('receive-changes',delta)
        // })

        //TODO: raspberry pi connect
    

    socket.on('disconnect',()=>{
        console.log('Disconnected! '+socket.id)
        if (socket.id === carSocketId[socket.id]){
            socket.broadcast.emit('car-offline',false)
            console.log('car disconnected')
            socket.off('car-socket-id',carSocketIdListener)
            socket.off('sending-result',sendingResultListener)
            delete carSocketId[socket.id]
        }
        
    })

    socket.on('error', (err) => {
        console.log(err)
    })
})

async function createInventory(content){
    return await Inventory.create(content)
}
async function createQueue(content){
    return await Queue.create(content)
}

async function deleteFirstItem(){
    return await Queue.deleteOne().sort({ createdAt: 1 })
        .exec((err, item) => {
            if (err) return (err)
        })
}

inventoryWatch.on('change',async (change)=>{
    if(change.operationType === 'insert') {
        // const task = change.fullDocument
        await pusher.trigger(
            channel,
            'inserted', 
            {
            // location: task.location
            }
        ).catch((error)=>{console.log(error)})
    } 
})

queueWatch.on('change', async (change) =>{
    if (change.operationType === 'delete'){
        await pusher.trigger(channel,'deleted', {})
        .catch((error)=>{console.log(error)})
    }
})