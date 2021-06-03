import React from 'react'
import SearchBar from './SearchBar'
import History from './History'
import Mapping from './Mapping'
import Status from './Status'
import PhotoDisplay from './PhotoDisplay'
import SidebarBox from './SidebarBox'
import {useState} from 'react'
export default function Homepage() {
    // const [photo, setPhoto] = useState()
    // const [status,setStatus] = useState()
    const [clickPhoto,setClickPhoto] = useState()
    // status = {status} photo={photo}
    // setStatus={setStatus} setPhoto={setPhoto}
    return (
        <>
        <SearchBar/>
        <div className="grid-container">
            <SidebarBox  setClickPhoto={setClickPhoto}/>
            <div className="grid-item">
                <Mapping />
                <PhotoDisplay clickPhoto={clickPhoto}/>
            </div>
            <div className="grid-item">
                <Status/>
                <History/>
            </div>
        </div>
        </>
    )
}
