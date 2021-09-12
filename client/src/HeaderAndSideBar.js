import React from 'react'
import Header from './Components/Header'
import Sidebar from './Components/Sidebar'

export default function HeaderAndSideBar({navBarState,setNavBarState,setClickGallery,setQueueOnClick,setStepsEnabled}) {
    return (
        <>
            <Header navBarState={navBarState} setNavBarState={setNavBarState} setStepsEnabled={setStepsEnabled}/>
            <Sidebar navBarState={navBarState} setClickGallery={setClickGallery} setQueueOnClick={setQueueOnClick}/>
        </>
    )
}
