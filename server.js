const express = require('express')
const mongoose = require('mongoose')
const cors = require('cors');
const Inventory = require('./model/inventory.model')
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

const PORT = process.env.PORT || 5000
const httpServer = require('http').createServer(app).listen(PORT)

app.use(cors())
app.use(express.json())


app.use('/login',loginRouter)
app.use('/inventory',inventoryRouter)
app.use('/sort',sortRouter)

if (process.env.NODE_ENV === "production"){
    app.use(express.static(path.join(__dirname,"client", "build")))

    app.get("*", (req, res) => {
        res.sendFile(path.join(__dirname,"client", "build", "index.html"));
    });
}

const io = require('socket.io')(httpServer,{
    cors:{
        origin: [`http://localhost:3000`],
        methods: ["GET", "POST"],
    },
})

io.on("connection",socket =>{
    console.log('server connected')

    socket.on('raspberry-send',delta =>{
        socket.broadcast.emit('receive-raspberry',delta)
    })

    socket.on('begin-search',delta =>{
        socket.broadcast.emit('sending-search', delta)
    })

    socket.on('sending-result',async delta =>{
        const content = {
            'location':delta['location'],
            'photo':delta['photo'],
            'userId':delta['userId']
        }
        await createInventory(content).catch(err => {console.log(err)})
        // console.log('server received ',delta['location'])
    })

    // const str = "testing this line"
    // socket.broadcast.emit('sending',str)

    // socket.on('get-user', async userId =>{
        // socket.on('send-changes',delta =>{
        //     // console.log("you see this ",delta)
        //     socket.broadcast.emit('receive-changes',delta)
        // })

        //TODO: user login with passport, query, raspberry pi connect

    socket.on('car-socket-id',(delta)=>{
        carSocketId[socket.id] = delta
    })

    socket.on('disconnect',()=>{
        console.log('Disconnected! '+socket.id)
        if (socket.id === carSocketId[socket.id]){
            socket.broadcast.emit('car-offline',false)
            console.log('true')
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



inventoryWatch.on('change',async (change)=>{
    if(change.operationType === 'insert') {
        const task = change.fullDocument
        await pusher.trigger(
            channel,
            'inserted', 
            {
            // location: task.location
            }
        ).catch((error)=>{console.log(error)})
    } 
})