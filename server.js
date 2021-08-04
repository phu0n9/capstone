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
    appId: process.env.PUSHER_APP_ID,
    key: process.env.PUSHER_KEY,
    secret: process.env.PUSHER_SECRET,
    cluster: process.env.PUSHER_CLUSTER,
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

io.on("connection",socket =>{
    socket.emit("popup",true)

    socket.on('begin-search',async delta =>{
        await createQueue(delta).then(()=>{
            socket.emit('popup',true)
        })
        .catch(err =>{console.log(err)})
    })

    socket.on('disconnect',()=>{
        console.log('Disconnected! '+socket.id)
    })

    socket.on('error', (err) => {
        console.log(err)
    })
})

// async function createInventory(content){
//     return await Inventory.create(content)
// }

async function createQueue(content){
    return await Queue.create(content)
}

// async function deleteFirstItem(){
//     return await Queue.deleteOne().sort({ createdAt: 1 })
//         .exec((err, item) => {
//             if (err) return (err)
//         })
// }

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
    else if(change.operationType === 'insert'){
        await pusher.trigger(channel,'inserted', {})
        .catch((error)=>{console.log(error)})
    }
})



