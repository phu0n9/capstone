import React from 'react'
import SearchBar from './SearchBar'
import DroneStatus from './DroneStatus'
import Mapping from './Mapping'
import CarStatus from './CarStatus'
import PhotoDisplay from './PhotoDisplay'
import SidebarBox from './SidebarBox'
import Setting from './Setting'
import {useState,useEffect} from 'react'
export default function Homepage() {
    const [clickPhoto,setClickPhoto] = useState()
    const [userId,setUserId] = useState()

    useEffect(()=>{
        setUserId(localStorage.getItem('userId'))
    },[])

    return (
        <>
        <span className="grid-container">
            <img src="schaeffler.png" alt="logo" className="logo"/>
            <SearchBar userId={userId}/>
            <Setting/>
            <span className="switch">
                <input type="checkbox"/>
                <span className="slider round"></span>
            </span>
        </span>
        <div className="grid-container">
            <SidebarBox  setClickPhoto={setClickPhoto}/>
            <div className="grid-item">
                <Mapping />
                <PhotoDisplay clickPhoto={clickPhoto}/>
            </div>
            <div className="grid-item">
                <CarStatus/>
                <DroneStatus/>
            </div>
        </div>
        </>
    )
}
