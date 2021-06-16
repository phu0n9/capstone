import React,{useEffect,useState} from 'react'
import {io} from 'socket.io-client'

export default function Status() {
    const [socket,setSocket] = useState()
    const [velocity,setVelocity] = useState()
    const [battery,setBattery] = useState()

    useEffect(() => {
        const s = io(`https://schaeffler.herokuapp.com/`)
        setSocket(s)
        return () =>{
            s.disconnect()  
        }
    }, [])  

    useEffect(() =>{
        if(socket == null) return

        const handler = (delta) =>{
            setVelocity(delta['velocity'])
            setBattery(delta['battery percentage'])
        }
        socket.on('receive-raspberry',handler)
        return () =>{
            socket.off('receive-raspberry',handler)
        }
    },[socket,velocity,battery])

    return (
        <div className="status-wrapper">
            <div>Velocity : {velocity} m/s</div>
            <div>Battery : {battery} %</div>
        </div>
    )
}
