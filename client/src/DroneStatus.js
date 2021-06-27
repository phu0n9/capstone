import React from 'react'

export default function DroneStatus() {

    return (
        <div className="droneStatus-wrapper">
            <div className="img-container">
                <img src="drone.png" alt="drone-img" className="img-oval"/>
                <div class="offline-dot"/>
            </div>
            <p>Drone is offline</p>
        </div>
    )
}
