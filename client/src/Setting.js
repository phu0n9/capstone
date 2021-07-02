import React from 'react'

export default function Setting() {
    return (
        <span className="setting-wrapper">
            <img src="black-bell.png" alt="noti" className="img-wrapper"/>
            <div class="dropdown">
                <img src="black-user.png" alt="user" className="img-wrapper"/>
                <div class="dropdown-content">
                    <p>Log out</p>
                </div>
            </div>
            <img src="black-settings.png" alt="setting" className="img-wrapper"/>
        </span>
    )
}
