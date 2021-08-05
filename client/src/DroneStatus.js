import React,{useState,useEffect} from 'react'
import Battery from './Components/Battery'
import Pusher from 'pusher-js'
require('dotenv').config()

export default function DroneStatus() {
    const [battery,setBattery] = useState(0)
    const [altitude,setAltitude] = useState("")
    const [connectTitle,setConnectTitle] = useState('offline')
    const [connection,setConnection] = useState(false)
    const [location,setLocation] = useState("")

    useEffect(() =>{
        const pusher = new Pusher(process.env.REACT_APP_PUSHER_KEY,{
            'cluster':process.env.REACT_APP_PUSHER_CLUSTER,
            forceTLS: true,
        })

        const messageChannel  = pusher.subscribe('droneMessage')
        messageChannel.bind('send',function(data){
            setBattery(data.battery)
            setAltitude(data.altitude)
            setConnection(true)
            setConnectTitle('online')
            setLocation(data.location)
        })
        return () => messageChannel.unbind()
    },[])

    return (
        <div className="droneStatus-wrapper">
            <Battery value={battery}/>
            <div className="img-container" style={{marginTop:"30px"}}>
                <img src="drone.png" alt="drone-img" className="img-oval"/>
                <div className={connection ? "offline-dot online-dot": "offline-dot"}/>
            </div>
            <p>Drone is {connectTitle}</p>
            <p>
                <img src="altitude.png" alt="altitude" className="icon-wrapper" />
                Altitude: {altitude}
            </p>
            <p>
            <img src="location.png" alt="location" className="icon-wrapper"/>
                Current Location: {location}
            </p>
        </div>
    )
}
