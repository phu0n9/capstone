import React,{useEffect,useState} from 'react'
import ReactSpeedometer from "react-d3-speedometer"
import Pusher from 'pusher-js'
require('dotenv').config()

export default function Status() {
    const [velocity,setVelocity] = useState(0)
    const [location,setLocation] = useState(undefined)
    const [connection,setConnection] = useState(false)
    const [connectTitle,setConnectTitle] = useState('offline')

    useEffect(() =>{
        const pusher = new Pusher(process.env.PUSHER_KEY,{
            'cluster':process.env.PUSHER_CLUSTER,
            encrypted:true
        })

        const messageChannel  = pusher.subscribe('carMessage')
        messageChannel.bind('send',function(data){
            setVelocity(data.velocity)
            setLocation(data.location)
            setConnection(true)
            setConnectTitle('online')
        })


        messageChannel.bind('connection',(status)=>{
            setConnection(status)
            setConnectTitle('offline')
            setVelocity(0)
            setLocation(undefined)
            console.log(status)
        })

        return () => {
            pusher.unsubscribe(messageChannel)
            pusher.disconnect()
        }
    },[])

    return (
        <div className="status-wrapper">
            <div className="img-container">
                <img src="car.png" alt="car-img" className="img-oval"/>
                <div className={connection ? "offline-dot online-dot": "offline-dot"}/>
            </div>
            <p>Car is {connectTitle}</p>
            <p>
                <img src="location.png" alt="location" className="icon-wrapper"/>
                Current location: {location}
            </p>
            <ReactSpeedometer
            width={150}
            height={100}
            maxValue={50}
            value={velocity}
            needleColor="red"
            startColor="green"
            segments={5}
            endColor="blue"
            />
        </div>

    )
}
