import React,{useState,useEffect} from 'react'
import { useAuth0 } from '@auth0/auth0-react'

export default function Sidebar({navBarState,setClickGallery,setQueueOnClick}) {
    const {isAuthenticated,logout} = useAuth0()
    const [currentURIState,setCurrentURIState] = useState(false)
    const [currentURI,setCurrentURI] = useState(window.location.pathname)

    useEffect(()=>{
        setCurrentURI(window.location.pathname)
        setCurrentURIState((currentURI === "/profile")? true: false)
    },[currentURI])

    return (
        <>
            <div className={navBarState ? "show l-navbar":"l-navbar"}>
                <nav className="nav">
                    <div> 
                        <div className="nav_logo"> 
                            <img src="https://image.flaticon.com/icons/png/512/2876/2876741.png" alt="drone" className="drone-icon"/>
                            <span className="nav_logo-name">Inventory Check</span> 
                        </div>
                        <div className="nav_list"> 
                            <a href="/" className={currentURIState ? "nav_link": "nav_link active"}>
                                <i className='bx nav_icon bx-grid-alt'></i>
                                <span className="nav_name">Dashboard</span>
                            </a> 
                            <a href="/profile" className={currentURIState ? "nav_link active": "nav_link"}>
                                <i className='bx nav_icon bx-user'></i>
                                <span className="nav_name">Profile</span>
                            </a> 
                       
                        </div>
                    </div> 
                    <div>
                        <button id="picture-gallery" className="nav_link option-btn" onClick={() => setClickGallery(true)}> 
                            <i className='bx bx-folder nav_icon'></i> 
                            <span className="nav_name">Picture Gallery</span> 
                        </button>
                        <button id="queue-command" className="nav_link option-btn" onClick={() => setQueueOnClick(true)}> 
                            <i className='bx bx-layer nav_icon'></i> 
                            <span className="nav_name">Queue</span> 
                        </button>
                        {isAuthenticated ? 
                        <a href={window.location.origin} className="nav_link" onClick={() => logout()}> 
                            <i className='bx bx-log-out nav_icon'></i> 
                            <span className="nav_name">Sign Out</span> 
                        </a> : ""}
                    </div>
                </nav>
            </div>
        </>
    )
}
