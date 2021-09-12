import React from 'react'
import Battery from './Components/Battery'
require('dotenv').config()

export default function DroneStatus({droneBattery,droneAltitude,droneLocation}) {
    
    return (
        <div className="device-container">
            <p className="device-status">DRONE STATUS</p>
            <Battery value={droneBattery}/> <br />
            <p className="status-title">
                <img src="altitude.png" alt="altitude" className="icon-wrapper" />
                Altitude:
            </p>
            <p className="status-value">{droneAltitude} m</p>
            <p className="status-title">
            <img src="location.png" alt="location" className="icon-wrapper"/>
                Current Location: 
            </p>
            <p className="status-value">{droneLocation}</p>
        </div>
    )
}
