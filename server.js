const express = require('express')
const mongoose = require('mongoose')
const cors = require('cors');
const Inventory = require('./model/inventory.model')
const User = require('./model/user.model')
const path = require('path')


require('dotenv').config()

const app = express()


const IP = 'localhost'
// const IP = '172.20.10.3'

const connection = mongoose.connection;
connection.once('open',() => {
    console.log("MongoDB database connection established successfully");
});

const uri = process.env.ATLAS_URI;

mongoose.connect(uri,
{
    useNewUrlParser:true,
    useCreateIndex:true,
    useUnifiedTopology: true,
    useFindAndModify: false
})

// app.use(cors())
// app.use(express.json())

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
    socket.on('raspberry-send',delta =>{
        socket.broadcast.emit('receive-raspberry',delta)
    })

    socket.on('sending-result',delta =>{
        socket.broadcast.emit('display-result',delta)
        console.log('server received ',delta['location'])
    })

    socket.on('get-user', async userId =>{
        // socket.on('send-changes',delta =>{
        //     // console.log("you see this ",delta)
        //     socket.broadcast.emit('receive-changes',delta)
        // })
        socket.on('begin-search',delta =>{
            socket.broadcast.emit('sending-search', delta)
            console.log('server sending ',delta)
            // io.emit('user-id',userId)    
        })
    

        //TODO: infinity scroll, connect database & load data, raspberry pi connect
       
        // const inventory = findOrCreateInventory(userId)

        // socket.join(userId)
        // socket.emit('load-inventory',inventory.data)

        // socket.on("save-data", async data =>{
        //     Inventory.findOneAndUpdate(userId,{data})
        //     .then(inventory => {
        //         inventory.data = data;
        //         inventory.save()
        //         console.log("userId= ",userId,"---",data," has been added")
        //     })
        //     .catch(
        //         err => {
        //             console.log(err)
        //         }
        //     )
        // })

    })

    console.log('server connected')

    socket.on('disconnect',function(){
        console.log('Disconnected!')
    })
})

const defaultValue = ""

async function findOrCreateInventory(id){
    if(id == null) return

    const filter = {userId:id}

    const inventory = await Inventory.find(filter)
    if(inventory) return inventory
    return await Inventory.create({data:defaultValue,userId:id})
}



