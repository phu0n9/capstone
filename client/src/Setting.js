import React from 'react'
import LogoutButton from './Components/LogoutButton'

export default function Setting() {
    return (
        <span className="setting-wrapper">
            <img src="upload.png" alt="noti" className="img-wrapper"/>
            <div className="dropdown">
                <img src="user.png" alt="user" className="img-wrapper"/>
                <div className="dropdown-content">
                    <LogoutButton/>
                </div>
            </div>
            <img src="queue.png" alt="queue" className="img-wrapper"/>
        </span>
    )
}
