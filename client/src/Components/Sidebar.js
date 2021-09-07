import React from 'react'
import { useAuth0 } from '@auth0/auth0-react'

export default function Sidebar({navBarState,setClickGallery,setQueueOnClick}) {
    const {isAuthenticated,logout} = useAuth0()

    const data = [
        { className:"bx-grid-alt",value:"Dashboard",link:'/'},
        { className:"bx-user",value:"Profile",link:'/profile'},
        { className:"bx-bar-chart-alt-2",value:"Statistic",link:'/statistics'}
    ]

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
                        { 
                            data.map((item,index) =>{
                                if(index === 0){
                                    return <a href={window.location.origin} className="nav_link active" key={index}>
                                        <i className={'bx nav_icon '+item.className} key={item.className}></i>
                                        <span className="nav_name" key={item.value}>{item.value}</span>
                                    </a> 
                                }
                                else{
                                    return <a href={item.link} className="nav_link" key={index}>
                                        <i className={'bx nav_icon '+item.className} key={item.className}></i>
                                        <span className="nav_name" key={item.value}>{item.value}</span>
                                    </a> 
                                }
                                
                            })
                        }
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
