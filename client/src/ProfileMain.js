import React,{useState} from 'react'
import HeaderAndSideBar from './HeaderAndSideBar'
import Profile from './Profile'
import Loading from './Components/Loading'
import {useAuth0} from '@auth0/auth0-react'

export default function ProfileMain() {
    const [queueOnClick,setQueueOnClick] = useState(false)
    const [navBarState,setNavBarState] = useState(false)
    const [clickGallery,setClickGallery] = useState(false)
    const {isLoading} = useAuth0()

    if (isLoading) {
        return <Loading />
    }

    return (
        <>
            <HeaderAndSideBar 
            navBarState={navBarState} setNavBarState={setNavBarState}
            setClickGallery={setClickGallery} setQueueOnClick={setQueueOnClick}/>
            <Profile clickGallery={clickGallery} setClickGallery={setClickGallery} queueOnClick={queueOnClick}/>
        </>
        
    )
}
