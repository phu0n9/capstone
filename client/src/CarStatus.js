import React,{useEffect,useState} from 'react'
import {io} from 'socket.io-client'
import ReactSpeedometer from "react-d3-speedometer"
import Pusher from 'pusher-js'

export default function Status() {
    const [socket,setSocket] = useState()
    const [velocity,setVelocity] = useState(0)
    const [location,setLocation] = useState(undefined)
    const [connection,setConnection] = useState(false)
    const [connectTitle,setConnectTitle] = useState('offline')
    const [state,setState] = useState(undefined)
    const heroku = 'https://schaeffler.herokuapp.com/'
    // 'http://localhost:5000/'
    useEffect(() => {
        const s = io('http://localhost:5000/')
        setSocket(s)
        return () =>{
            s.disconnect()  
        }
    }, [])  

    // useEffect(() =>{
    //     if(socket == null) return
    //     const handler = (delta) =>{
    //         setVelocity(delta['velocity'])
    //         setLocation(delta['current location'])
    //         setConnection(true)
    //         setConnectTitle('online')

    //     }
    //     socket.on('receive-raspberry',handler)

    //     return () =>{
    //         socket.off('receive-raspberry',handler)
    //     }
    // },[socket,velocity,location])

    // useEffect(() =>{
    //     if (socket == null) return 

    //     const handler = (delta) =>{
    //         setConnection(delta)
    //         setConnectTitle('offline')
    //         setVelocity(0)
    //         setLocation(undefined)
    //     }
    //     socket.on('car-offline',handler)
    //     return () =>{
    //         socket.off('car-offline',handler)
    //     }
    // },[socket,connection])


    useEffect(() =>{
        const pusher = new Pusher('2ccb32686bdc0f96f50a',{
            'cluster':'ap1',
            encrypted:true
        })

        const messageChannel  = pusher.subscribe('carMessage')
        messageChannel.bind('send',function(data){
            // console.log("this "+data.message)
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
            <div>{state}</div>
        </div>

    )
}
