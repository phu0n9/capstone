import React from 'react'

export default function Setting() {
    return (
        <span className="setting-wrapper">
            <img src="upload.png" alt="noti" className="img-wrapper"/>
            <div className="dropdown">
                <img src="user.png" alt="user" className="img-wrapper"/>
                <div className="dropdown-content">
                    <p>Log out</p>
                </div>
            </div>
            <img src="setting.png" alt="setting" className="img-wrapper"/>
        </span>
    )
}
