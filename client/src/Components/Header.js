import React from 'react'
import { useAuth0 } from '@auth0/auth0-react'
import { useHistory } from "react-router-dom";

export default function Header({navBarState,setNavBarState,setStepsEnabled}) {
    const {user} = useAuth0()
    const userPicture = user.picture
    const history = useHistory()

    const handleSupportBtn = () => {
        if(window.location.pathname === "/profile"){
            history.push("/")
        }
        else{
            setStepsEnabled(true)
        }
    }

    return (
        <header className={navBarState? "body-pd header":"header"} >
            <div className="header_toggle">
                <i className={navBarState ? "bx-x bx bx-menu":'bx bx-menu'} value={navBarState} onClick={()=> { setNavBarState(!navBarState)}}></i> 
            </div>
            <img src="schaeffler.png" alt="navLogo" className="header_img"/>

            <button className="support-btn" onClick={handleSupportBtn}>
                <img src="help.png" alt="help" className="support-icon"/>
            </button>
            
            <div className="header_profile"> 
                <img src={userPicture} alt="user"/> 

            </div>
        </header>
    )
}
