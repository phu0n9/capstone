import React,{useState,useEffect} from 'react'
import SearchBar from './SearchBar'
import Status from './Status'
import Pusher from 'pusher-js'

export default function ControlBox({buttonSubmit,setButtonSubmit,droneConnection,carConnection,setQueueOnClick}) {
    const [deviceCheck,setDeviceCheck] = useState(false)
    const [carVelocity,setCarVelocity] = useState(0)
    const [carLocation,setCarLocation] = useState("undefined")

    const [droneBattery,setDroneBattery] = useState(0)
    const [droneAltitude,setDroneAltitude] = useState(0)
    const [droneLocation,setDroneLocation] = useState("undefined")

    useEffect(() =>{
        const pusher = new Pusher(process.env.REACT_APP_PUSHER_KEY,{
            'cluster':process.env.REACT_APP_PUSHER_CLUSTER,
            forceTLS: true,
        })

        const droneChannel  = pusher.subscribe('droneMessage')
        droneChannel.bind('send',function(data){
            setDroneBattery(data.battery)
            setDroneAltitude(data.altitude)
            setDroneLocation(data.location)
        })

        const carChannel  = pusher.subscribe('carMessage')
        carChannel.bind('send',function(data){
            setCarVelocity(data.velocity)
            setCarLocation(data.location)
        })
        return () => {
            pusher.unsubscribe(droneChannel)
            pusher.unsubscribe(carChannel)
            pusher.disconnect()
        }
    },[])

    useEffect(()=>{
        if(!carConnection && !droneConnection){
            setDroneBattery(0)
            setDroneAltitude(0)
            setDroneLocation("undefined")
            setCarVelocity(0)
            setCarLocation("undefined")
        }
    },[carConnection,droneConnection])

    return (
        <>
            <div className="wrapper">
                <div className="search-sidebar">
                    <SearchBar setDeviceCheck={setDeviceCheck} deviceCheck={deviceCheck} 
                    setButtonSubmit={setButtonSubmit} buttonSubmit={buttonSubmit}
                    setQueueOnClick={setQueueOnClick}/>
                </div>
                <div className="status-wrapper">
                    <Status setDeviceCheck={setDeviceCheck} deviceCheck={deviceCheck}
                     carLocation={carLocation} carVelocity={carVelocity} carConnection={carConnection}
                     droneBattery={droneBattery} droneAltitude={droneAltitude} 
                     droneLocation={droneLocation} droneConnection={droneConnection}/>
                </div>
            </div>
        </>
    )
}
