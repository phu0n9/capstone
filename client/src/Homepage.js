import React from 'react'
import SearchBar from './SearchBar'
import DroneStatus from './DroneStatus'
import Mapping from './Mapping'
import CarStatus from './CarStatus'
import PhotoDisplay from './PhotoDisplay'
import SidebarBox from './SidebarBox'
import Setting from './Setting'
import Loading from './Components/Loading'
import {useState} from 'react'
import {useAuth0} from '@auth0/auth0-react'


export default function Homepage() {
    const [clickPhoto,setClickPhoto] = useState()
    const [onClick,setOnClick] = useState(false)
    const {isLoading} = useAuth0()

    if (isLoading) {
        return <Loading />
    }

    return (
        <>
        <span className="grid-container">
            <img src="schaeffler-logo.jpg" alt="logo" className="logo"/>
            <SearchBar/>
            <Setting/>
            {/* <label class="switch">
                <input type="checkbox" checked/>
                <span class="slider round"></span>
            </label> */}
        </span>
        
        
        <div className="grid-container">
            <SidebarBox  setClickPhoto={setClickPhoto}/>
            <div className="grid-item">
                <Mapping onClick={onClick} setOnClick={setOnClick}/>
                <PhotoDisplay clickPhoto={clickPhoto} onClick={onClick} setOnClick={setOnClick}/>
            </div>
            <div className="grid-item">
                <CarStatus/>
                <DroneStatus/>
            </div>
        </div>
        <footer>
            <p>
            © 2021 Schaeffler All rights reserved.<br/>
            </p>
        </footer>
        </>
    )
}
