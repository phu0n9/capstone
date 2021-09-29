#!/usr/bin/env nodemon
'use strict'
const Pusher = require('pusher-js')
const rosnodejs = require('rosnodejs')
const std_msgs = rosnodejs.require('std_msgs').msg
const path = require('path')

require('dotenv').config({path:'//home/tuan/catkin_ws/src/beginner_tutorials/scripts/.env'})

const pusher = new Pusher(process.env.PUSHER_KEY,{
    'cluster':process.env.PUSHER_CLUSTER,
    encrypted:true
})
const fs = require('fs')
const Queue = require('./models/queue.model')
const Inventory = require('./models/inventory.model')
let queueId = ""
let start = 0

const uri = process.env.ATLAS_URI
const mongoose = require('mongoose')

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

function sleep(milliseconds){
    const date = Date.now()
    let currentDate = null
    do{
        currentDate = Date.now()
    } while(currentDate - date < milliseconds)
}

function removeAllPictures(){
    const directory = __dirname+"/picture"
    fs.readdir(directory, (err, files) => {
        if (err) throw err;
      
        for (const file of files) {
          fs.unlink(path.join(directory, file), err => {
            if (err) throw err;
          })
        }
      })
}

function receivedKeyword(){
    const msg = new std_msgs.String() 
    rosnodejs.initNode('/client_js_node')
    .then((rosNode) =>{
        const searchChannel  = pusher.subscribe('search')
        let pub = rosNode.advertise('/client_received_key',std_msgs.String,{queueSize:5,tcpNoDelay:true,latching:true})
        sleep(1000)
        setInterval(()=>{
                if (start === 0){
                    msg.data = "start"
                    pub.publish(msg)
                }
                else if (start === 1){
                    msg.data = "restart"
                    pub.publish(msg)
                }
            },10000
        )

        searchChannel.bind('keyword',function(data){
            rosnodejs.log.info(data)
            if (data !== ""){
                start = 2
                queueId = data
                rosnodejs.log.info("data: " + data)
                Queue.findById(data)
                .then(queue =>{
                    const msg = new std_msgs.String() 
                    msg.data = queue.keyword
                    pub.publish(msg)
                    rosnodejs.log.info("client sent "+msg)
                })
                .catch(err =>{console.log(err)})
            }
        })

        rosNode.subscribe('/qrbarcode', std_msgs.String,
        (message) => { // define callback execution
            rosnodejs.log.info(message.data)
            if (message.data === "no rack"){
                msg.data = "armarker"
                pub.publish(msg)
            }
            else if(message.data !== "qrbarcodeStart"){
                Queue.findById(queueId)
                .then(queue =>{
                    fs.readFile(path.join(__dirname, '/picture/', message.data), function(error, data) {
                        if (error) {
                            console.log(error)
                        } else {
                            let base64 = data.toString('base64')
                            rosnodejs.log.info(path.join(__dirname, '/picture/', message.data))
                            var result = {
                                "location":queue.keyword,
                                "userId" : queue.userId,
                                "photo": base64
                            }
                            Inventory.create(result)            
                            .then(()=>{
                                Queue.findByIdAndDelete(queueId)
                                .then(()=>{console.log("Queue deleted")})
                                .catch((err)=>{console.log(err)})
                                queueId = ""
                                removeAllPictures()
                                msg.data = "armarker"
                                pub.publish(msg)
                            })
                            .catch(err => console.log(err))
                        }
                    })
                })
                .catch(err => console.log(err))
            }
        })
        
        rosNode.subscribe('/ArMarker',std_msgs.String,
        (message) =>{
            if(message.data === "restart"){
                start = 1
            }
        })
        
    })
}

if (require.main === module){
    receivedKeyword()
}
