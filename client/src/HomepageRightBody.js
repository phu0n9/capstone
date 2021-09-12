import React,{useState,useEffect} from 'react'
import Mapping from './Mapping'
import ControlBox from './ControlBox'
import PictureGallery from './PictureGallery'
import Queue from './Components/Queue'
import ReactNotification from 'react-notifications-component'
import {store} from 'react-notifications-component'
import 'react-notifications-component/dist/theme.css'
import Pusher from 'pusher-js'
import { Steps } from "intro.js-react";

export default function RightBody({setClickGallery,clickGallery,setQueueOnClick,queueOnClick,setStepsEnabled,stepsEnabled}) {
    const [buttonSubmit,setButtonSubmit] = useState(false)
    const [droneConnection,setDroneConnection] = useState(false)
    const [carConnection,setCarConnection] = useState(false)
    const [available,setAvailable] = useState(undefined)
    const [executeOnClick,setExecuteOnClick] = useState(false)
    const [rowDes,setRowDes] = useState(0)
    const [rackDes,setRackDes] = useState(0)
    const [checkRack,setCheckRack] = useState(undefined)
    const [droneStatus,setDroneStatus] = useState("")
    const [scanned,setScanned] = useState(false)
    const [rowClick,setRowClick] = useState(undefined)
    const [rackClick,setRackClick] = useState(undefined)
    const [keyword,setKeyword] = useState(undefined)

    //Intro js
    const initialStep = 0
    const steps = ([{
            element: ".header",
            intro: "Welcome to Schaeffler Inventory Checking."
          },
          {
            element: ".search-sidebar",
            intro: "Search your item here. Example: 5.6.7 where 5 is rack,6 is column,7 is row."
          },
          {
            element: ".status-wrapper",
            intro: "Remember to check the device before submit."
          },
          {
            element: ".mapping",
            intro: "Click to location to see existing search in the map."
          },
          {
            element: "#picture-gallery",
            intro: "Existing picture would be shown here."
          },
          {
            element: "#queue-command",
            intro: "List of item command when device is not online would be shown here."
          },
          {
            element: ".bx-user",
            intro: "User profile"
          },
          {
              element: ".support-btn",
              intro: "End of tour.Click this to start the tour again."
          }
    ])

    useEffect(() =>{
        const pusher = new Pusher(process.env.REACT_APP_PUSHER_KEY,{
            'cluster':process.env.REACT_APP_PUSHER_CLUSTER,
            forceTLS: true,
        })

        const carMessage  = pusher.subscribe('carMessage')
        carMessage.bind('send',function(data){
            setCarConnection(JSON.parse(data.carConnection))
        })
        const droneMessage  = pusher.subscribe('droneMessage')
        droneMessage.bind('send',function(data){
            setDroneConnection(JSON.parse(data.droneConnection))
        })

        const availableMessage  = pusher.subscribe('availability')
        availableMessage.bind('keyword',function(data){
            setAvailable(JSON.parse(data))
        })

        const statusChannel = pusher.subscribe('status')
        statusChannel.bind('rack',function(data){
            setCheckRack(JSON.parse(data))
        })

        statusChannel.bind('land',function(data){
            setDroneStatus(data.landingStatus)
        })

        const offlineChannel = pusher.subscribe('message')
        offlineChannel.bind('connection',function(data){
            setCarConnection(data)
            setDroneConnection(data)
        })

        const scanChannel = pusher.subscribe('status')
        scanChannel.bind('search',function(data){
            setScanned(JSON.parse(data))
        })

        return () => {
            pusher.unsubscribe(carMessage)
            pusher.unsubscribe(droneMessage)
            pusher.unsubscribe(availableMessage)
            pusher.unsubscribe(statusChannel)
            pusher.unsubscribe(offlineChannel)
            pusher.unsubscribe(scanChannel)
            pusher.disconnect()
        }
    },[])

    function getNotification(title,message,messageType,duration,width){
        store.addNotification({
            title:title,
            message:message,
            type: messageType,
            container:"bottom-right",
            insert:"bottom",
            animationIn:["animated","fadeIn"],
            animationOut:["animated","fadeOut"],
            dismiss: {
                duration: duration,
                onScreen: true
            },
            width:width
        })              
    }

    useEffect(()=>{
        if(buttonSubmit === true && carConnection === false && droneConnection === false){
            getNotification("Warning","Device is not online but your search will be saved.","warning",10000,500)
            setQueueOnClick(true)
        }
        else if(droneConnection && carConnection){
            getNotification("Device Status","Car and Drone are online","success",5000,400)       
        }
    },[buttonSubmit,carConnection,droneConnection,setQueueOnClick])

    useEffect(() =>{
        if(available === false && executeOnClick){
            getNotification("Device Status","Process are being executed. Try again later!","danger",7000,400)      
            setExecuteOnClick(false) 
        }
        else if(available === undefined && executeOnClick){
            getNotification("Device Status","Make sure to turn on the device.","danger",5000,400)      
            setExecuteOnClick(false) 
        }
        else if(available === true && carConnection && droneConnection){
            getNotification("Device Status","Search can be performed now.","success",5000,400)       
        }
        else if(checkRack === false){
            getNotification("Result","You have placed device at the wrong rack.","danger",10000,400) 
            setCheckRack(undefined)      
        }
        else if(droneStatus !== ""){
            getNotification("Drone status","Drone is "+droneStatus,"info",10000,400)       
        }
        else if(droneStatus === "landed"){
            setDroneStatus("")
            setRackDes(0)
            setRowDes(0)
        }
        else if(scanned){
            getNotification("Device status","Drone went to right position","success",10000,400)       
        }
        console.log("available "+available)
    },[available,executeOnClick,checkRack,droneStatus,droneConnection,carConnection,scanned])

    const handleOnExit = () => {
        setStepsEnabled(false)
    }

    useEffect(() =>{
        if(rowClick && rackClick){
            setClickGallery(true)
            setKeyword(rackClick+"."+rowClick)
        }
    },[rowClick,rackClick,setClickGallery])

    return (
        <>
            <ControlBox buttonSubmit={buttonSubmit} setButtonSubmit={setButtonSubmit} droneConnection={droneConnection} carConnection={carConnection} setQueueOnClick={setQueueOnClick}/>
            <Mapping rowDes={rowDes} rackDes={rackDes} available={available} setRowClick={setRowClick} setRackClick={setRackClick}/>
            {clickGallery ? <PictureGallery setClickGallery={setClickGallery} setRackClick={setRackClick} setRowClick={setRowClick} keyword={keyword} setKeyword={setKeyword}/>: ""}
            {queueOnClick ? <Queue setQueueOnClick={setQueueOnClick} available={available} setExecuteOnClick={setExecuteOnClick} setRackDes={setRackDes} setRowDes={setRowDes}/>: ""}
            <ReactNotification/>
            <Steps
            enabled={stepsEnabled}
            steps={steps}
            initialStep={initialStep}
            onExit={handleOnExit}
            />
        </>
    )
}
