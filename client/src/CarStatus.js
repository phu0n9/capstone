import React,{useEffect,useState} from 'react'
import {io} from 'socket.io-client'
import ReactSpeedometer from "react-d3-speedometer"

export default function Status() {
    const [socket,setSocket] = useState()
    const [velocity,setVelocity] = useState(0)
    const [location,setLocation] = useState(undefined)
    const [connection,setConnection] = useState(false)
    const [connectTitle,setConnectTitle] = useState('offline')
    const heroku = 'https://schaeffler.herokuapp.com/'
    
    useEffect(() => {
        const s = io('http://localhost:5000/')
        setSocket(s)
        return () =>{
            s.disconnect()  
        }
    }, [])  

    useEffect(() =>{
        if(socket == null) return

        const handler = (delta) =>{
            setVelocity(delta['velocity'])
            setLocation(delta['current location'])
            setConnection(true)
            setConnectTitle('online')
        }
        socket.on('receive-raspberry',handler)
        return () =>{
            socket.off('receive-raspberry',handler)
        }
    },[socket,velocity,location])

    useEffect(() =>{
        if (socket == null) return 

        const handler = (delta) =>{
            setConnection(delta)
            setConnectTitle('offline')
            setVelocity(0)
            setLocation(undefined)
        }
        socket.on('car-offline',handler)
        return () =>{
            socket.off('car-offline',handler)
        }
    },[socket,connection])

    return (
        <div className="status-wrapper">
            <div className="img-container">
                <img src="car.png" alt="car-img" className="img-oval"/>
                <div className={connection ? "offline-dot online-dot": "offline-dot"}/>
            </div>
            <p>Car is {connectTitle}</p>
            <p>Current location: {location}</p>
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
