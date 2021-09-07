import React from 'react'

export default function carStatus({carLocation,carVelocity}) {
    require('dotenv').config()

    return (
        <div className="device-container">
            <p className="device-status">CAR STATUS</p>
            <p className="status-title">
                <img src="location.png" alt="location" className="icon-wrapper"/>
                Current location:
            </p>
            <p className="status-value">{carLocation}</p>
            <p className="status-title">
                <img src="speedometer.png" alt="speedometer" className="icon-wrapper"/>
                Velocity:
            </p>
            <p className="status-value"> {carVelocity} m/s</p>
        </div>

    )
}
