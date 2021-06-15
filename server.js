const express = require('express')
const mongoose = require('mongoose')
const cors = require('cors');
const Inventory = require('./model/inventory.model')
const path = require('path')

require('dotenv').config()

const app = express()

const connection = mongoose.connection
connection.once('open',() => {
    console.log("MongoDB database connection established successfully")
})
const inventoryWatch = Inventory.watch()

const uri = process.env.ATLAS_URI

mongoose.connect(uri,
{
    useNewUrlParser:true,
    useCreateIndex:true,
    useUnifiedTopology: true,
    useFindAndModify: false
})

app.use(cors())
app.use(express.json())

const loginRouter = require('./routes/userLogin')

app.use('/login',loginRouter)

const inventoryRouter = require('./routes/inventoryLoading')
app.use('/inventory',inventoryRouter)

const PORT = process.env.PORT || 5000
const httpServer = require('http').createServer(app).listen(PORT)
const io = require('socket.io')(httpServer,{
    cors:{
        origin: [`http://localhost:3000`],
        methods: ["GET", "POST"],
    },
})

if (process.env.NODE_ENV === "production"){
    app.use(express.static(path.join(__dirname,"client", "build")))

    app.get("*", (req, res) => {
        res.sendFile(path.join(__dirname,"client", "build", "index.html"));
    });
}

io.on("connection",socket =>{
    console.log('server connected')

    socket.on('raspberry-send',delta =>{
        socket.broadcast.emit('receive-raspberry',delta)
    })

    socket.on('begin-search',delta =>{
        console.log('server sending ',delta)
        socket.broadcast.emit('sending-search', delta)
    })

    socket.on('sending-result',async delta =>{
        const content = {
            'location':delta['location'],
            'photo':delta['photo'],
            'userId':delta['userId']
        }
        await createInventory(content).catch(err => {console.log(err)})
        inventoryWatch.on('change',()=>{
            console.log('sending to front end')
            socket.broadcast.emit('sending','hello')
        })


        // console.log('server received ',delta['location'])
    })

    // const str = "testing this line"
    // socket.broadcast.emit('sending',str)

    // socket.on('get-user', async userId =>{
        // socket.on('send-changes',delta =>{
        //     // console.log("you see this ",delta)
        //     socket.broadcast.emit('receive-changes',delta)
        // })

        //TODO: user login with token, infinity scroll, connect database & load data, raspberry pi connect

    socket.on('disconnect',function(){
        console.log('Disconnected!')
    })
})

async function createInventory(content){
    return await Inventory.create(content)
}