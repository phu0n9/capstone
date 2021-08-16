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
// const loginRouter = require('./routes/userLogin')
const inventoryRouter = require('./routes/inventoryLoading')
const sortRouter = require('./routes/sort')
const popupRouter = require('./routes/popup')

const PORT = process.env.PORT || 5000
const httpServer = require('http').createServer(app).listen(PORT)

app.use(cors())
app.use(express.json())

// app.use('/login',loginRouter)
app.use('/inventory',inventoryRouter)
app.use('/sort',sortRouter)
app.use('/queue',popupRouter)
//-------------------------------------------------------------END OF ROUTER-----------------------------------------
//Import Auth0
var jwt = require('express-jwt')
var jwks = require('jwks-rsa')

var jwtCheck = jwt({
    secret: jwks.expressJwtSecret({
        cache: true,
        rateLimit: true,
        jwksRequestsPerMinute: 5,
        jwksUri: process.env.JWKS_URI
  }),
  audience: process.env.JWT_AUDIENCE,
  issuer: process.env.JWT_ISSUER,
  algorithms: ['RS256']
})

app.use(jwtCheck)

app.get('/protected', async (req, res) => {
    try{
        const accessToken = req.headers.authorization.split(' ')[1]
        const response = await axios.get(process.env.JWT_ISSUER+'userInfo',{
            headers:{
                authorization: `Bearer ${accessToken}`
            }
        })
        res.send(response.data.sub)
    }
    catch(error){
        console.log(error.message)
    }
})

app.use((req,res,next) => {
    const error = new Error("Not found")
    error.status = 404
    next(error)
})

app.use((error,req,res,next)=>{
    const status = error.status || 500
    const message = error.message || 'Internal Server Error'
    res.status(status).send(message)
})

//-------------------------------------------------------------END OF AUTH0------------------------------------------
// Production mode: Heroku
if (process.env.NODE_ENV === "production"){
    app.use(express.static(path.join(__dirname,"client", "build")))

    app.get("*", (req, res) => {
        res.sendFile(path.join(__dirname,"client", "build", "index.html"));
    })
}
//------------------------------------------------------------END OF CONFIGURE PRODUCTION MODE-----------------------

// Import Socketio
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
async function createQueue(content){
    return await Queue.create(content)
}
//----------------------------------------------------------END OF IMPORT SOCKETIO-----------------------------------

// async function createInventory(content){
//     return await Inventory.create(content)
// }

// async function deleteFirstItem(){
//     return await Queue.deleteOne().sort({ createdAt: 1 })
//         .exec((err, item) => {
//             if (err) return (err)
//         })
// }

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



