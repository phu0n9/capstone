const express = require('express')
const mongoose = require('mongoose')
const cors = require('cors')
const axios = require('axios')
const Inventory = require('./model/inventory.model')
const Queue = require('./model/queue.model')

const path = require('path')
const Pusher = require('pusher')

require('dotenv').config()

const app = express()

const inventoryWatch = Inventory.watch()
const queueWatch = Queue.watch()

//----------------------------------------------------------------END OF IMPORT---------------------------------------
// Import MongoDB and Moongoose
const uri = process.env.ATLAS_URI

const connection = mongoose.connection

mongoose.connect(uri,{
    useNewUrlParser:true,
    useCreateIndex:true,
    useUnifiedTopology: true,
    useFindAndModify: false
})

connection.once('open',() => {
    console.log("MongoDB database connection established successfully")
})

connection.on('error', console.error.bind(console, 'Connection Error:'))

//--------------------------------------------------------------END OF IMPORT MONGOOSE---------------------------------

// Import Pusher
const pusher = new Pusher({
    appId: process.env.PUSHER_APP_ID,
    key: process.env.PUSHER_APP_KEY,
    secret: process.env.PUSHER_APP_SECRET,
    cluster: process.env.PUSHER_APP_CLUSTER,
    useTLS: true
})
const channel = 'tasks'

//------------------------------------------------------END OF IMPORT PUSHER -----------------------------------

//Router
app.use(cors())
app.use(express.json())

const PORT = process.env.PORT || 5000
const httpServer = require('http').createServer(app).listen(PORT)

const inventoryRouter = require('./routes/inventoryLoading')
const popupRouter = require('./routes/popup')
const searchRouter = require('./routes/search')
const profileRouter = require('./routes/profile')

app.use('/inventory',inventoryRouter)
app.use('/queue',popupRouter)
app.use('/search',searchRouter)
app.use('/profile',profileRouter)

//-------------------------------------------------------------END OF ROUTER-----------------------------------------

// Import cors
require('socket.io')(httpServer,{
    cors:{
        origin: [`http://localhost:3000`],
        methods: ["GET", "POST"],
    },
})

//----------------------------------------------------------END OF IMPORT CORS-----------------------------------

// Mongoose Listening Stream
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
    else if (change.operationType === 'delete'){
        await pusher.trigger(channel,'itemDeleted', {})
        .catch((error)=>{console.log(error)})
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

//-------------------------------------------------------END OF MONGOOSE LISTENING STREAM----------------------------

// Production mode: Heroku
if (process.env.NODE_ENV === "production"){
    app.use(express.static(path.join(__dirname,"client", "build")))

    app.get("*", (req, res) => {
        res.sendFile(path.join(__dirname,"client", "build", "index.html"));
    })
}
//------------------------------------------------------------END OF CONFIGURE PRODUCTION MODE-----------------------


