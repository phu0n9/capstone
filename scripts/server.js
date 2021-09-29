#!/usr/bin/env nodemon
'use strict';
/**
 * This example demonstrates simple receiving of messages over the ROS system.
 */

const Pusher = require('pusher')
const rosnodejs = require('rosnodejs')
const std_msgs = rosnodejs.require('std_msgs').msg
const path = require('path')

require('dotenv').config({path:'//home/tuan/catkin_ws/src/beginner_tutorials/scripts/.env'})

const pusher = new Pusher({
    appId: process.env.PUSHER_APP_ID,
    key: process.env.PUSHER_KEY,
    secret: process.env.PUSHER_SECRET,
    cluster: process.env.PUSHER_CLUSTER,
    useTLS: true
})

function listener() {
  // Register node with ROS master
    rosnodejs.initNode('/server_node')
    .then((rosNode) => {
        // Create ROS subscriber on the 'chatter' topic expecting String messages
        rosNode.subscribe('/carTalker', std_msgs.String,
        (message) => { // define callback execution 
          pusher.trigger("carMessage", "send", JSON.parse(message.data)).catch(error =>{console.log(error)})
        })
        rosNode.subscribe('/droneTalker', std_msgs.String,
        (message) => { // define callback execution
          pusher.trigger("droneMessage", "send", JSON.parse(message.data)).catch(error =>{console.log(error)})
        })
        rosNode.subscribe('/ArMarker', std_msgs.String,
        (message) => { // define callback execution
          if (message.data == "marker found"){
            pusher.trigger("status", "land", {landingStatus:"landed"})
            .catch(error =>{console.log(error)})
            rosNode.subscribe('/client_received_key', std_msgs.String,(message) =>{
              rosnodejs.log.info('I heard keyword message: [' + message.data + ']')
              if(message.data === "restart"){
                pusher.trigger("availability","keyword","true")
                .catch(error =>{console.log(error)})
              }
            })
          }
        })
        rosNode.subscribe('/qrbarcode', std_msgs.String,(message)=>{
          if(message.data === "no rack"){
            pusher.trigger("status","rack","false").catch(error =>{console.log(error)})
            pusher.trigger("availability","keyword","false").catch(error =>{console.log(error)})
          }
          else{
            pusher.trigger("status","search","true").catch(error =>{console.log(error)})
            pusher.trigger("availability","keyword","false").catch(error =>{console.log(error)})
          }
        })
       rosNode.subscribe('/client_received_key', std_msgs.String,(message) =>{
          rosnodejs.log.info('I heard keyword message: [' + message.data + ']')
          if(message.data === "start" ){
            pusher.trigger("availability","keyword","true").catch(error =>{console.log(error)})     
          }
          else if (message.data === "armarker"){
            pusher.trigger("status", "land", {landingStatus:"down"}).catch(error =>{console.log(error)})
            pusher.trigger("availability","keyword","false").catch(error =>{console.log(error)})
          }
          else if(message.data !== "restart"){
            pusher.trigger("status", "land", {landingStatus:"up"}).catch(error =>{console.log(error)})
            pusher.trigger("availability","keyword","false").catch(error =>{console.log(error)})
          }
        })
    })

}

process.on('SIGINT',async()=>{
  rosnodejs.on('shutdown',async () =>{
    await pusher.trigger("message","connection",false).then(() => {
      // Parse the response body as JSON
      rosnodejs.log.info("shutdown successfully")})
    .catch(error =>{console.log(error)})
    await pusher.trigger("availability","keyword","false").catch(error =>{console.log(error)})
    rosnodejs.log.info("shutdown server")
  })
})



if (require.main === module) {
  // Invoke Main Listener Function
  listener()    
}