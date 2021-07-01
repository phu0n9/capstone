import React,{useState,useEffect} from 'react'
import {io} from 'socket.io-client'
import Battery from './Components/Battery'

export default function DroneStatus() {
    const [socket,setSocket] = useState()
    const [battery,setBattery] = useState(0)
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
            setBattery(delta['battery'])
        }
        socket.on('receive-raspberry',handler)
        return () =>{
            socket.off('receive-raspberry',handler)
        }
    },[socket,battery])


    return (
        <div className="droneStatus-wrapper">
            <Battery value={battery}/>
            <div className="img-container" style={{marginTop:"30px"}}>
                <img src="drone.png" alt="drone-img" className="img-oval"/>
                <div class="offline-dot"/>
            </div>
            <p>Drone is offline</p>
        </div>
    )
}
