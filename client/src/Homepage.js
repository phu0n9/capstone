import React from 'react'
import Loading from './Components/Loading'
import {useState} from 'react'
import {useAuth0} from '@auth0/auth0-react'
import HomepageRightBody from './HomepageRightBody'
import HeaderAndSideBar from './HeaderAndSideBar'

export default function Homepage() {
    const [queueOnClick,setQueueOnClick] = useState(false)
    const [navBarState,setNavBarState] = useState(false)
    const [clickGallery,setClickGallery] = useState(false)
    const {isLoading} = useAuth0()
    const [stepsEnabled,setStepsEnabled] = useState(true)

    if (isLoading) {
        return <Loading />
    }

    return (
        <>
            <HeaderAndSideBar 
            navBarState={navBarState} setNavBarState={setNavBarState} 
            setClickGallery={setClickGallery} setQueueOnClick={setQueueOnClick} setStepsEnabled={setStepsEnabled}/>
            <HomepageRightBody 
            clickGallery={clickGallery} setClickGallery={setClickGallery} setQueueOnClick={setQueueOnClick} queueOnClick={queueOnClick} stepsEnabled={stepsEnabled} setStepsEnabled={setStepsEnabled}/>
        </>
    )
}
