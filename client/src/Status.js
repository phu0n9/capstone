import React,{useState} from 'react'
import CarStatus from './CarStatus'
import DroneStatus from './DroneStatus'

export default function Status({setDeviceCheck,deviceCheck,carVelocity,carLocation,droneLocation,droneAltitude,droneBattery,droneConnection,carConnection}) {
    const [clickMore,setClickMore] = useState(false)
    const handleDeviceCheck = () =>{
        setDeviceCheck(true)
    }
    return (
        <>
            <p className="title-text">DEVICE STATUS</p>
            <div className="device-wrapper">
                <p className="sort-title">Sort by:</p>
                <select name="sort" className="search-box status-sort">
                    <option value="name" defaultValue>Name</option>
                    <option value="status">Status</option>
                </select>
                <div className="device-list">
                    <p className="sort-status">Prototype 1</p>
                    <img src={droneConnection && carConnection ? "wifi-connected.png":"wifi-disconnected.png"} alt="wifi" className="wifi-icon"/>
                    <input type="radio" className="device-select" onChange={handleDeviceCheck} checked={deviceCheck}/>
                    {clickMore ?   
                        <div className="devices">
                            <CarStatus carLocation={carLocation} carVelocity={carVelocity}/>
                            <DroneStatus droneBattery={droneBattery} droneAltitude={droneAltitude} droneLocation={droneLocation}/>
                        </div>
                    : ""}
                  
                    <button className="more-btn" value={clickMore} onClick={() => setClickMore(!clickMore)}><img src={clickMore ? "caret-up.png":"caret-down.png"} alt="more-img" className="down-icon"/></button>
                </div>
            </div>
        </>
    )
}
