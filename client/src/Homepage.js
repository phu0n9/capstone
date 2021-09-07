import React from 'react'
import Loading from './Components/Loading'
import {useState} from 'react'
import {useAuth0} from '@auth0/auth0-react'
import HomepageRightBody from './HomepageRightBody'
import HeaderAndSideBar from './HeaderAndSideBar'
import Profile from './Profile'
import {
    BrowserRouter as Router,
    Switch,
    Route,
} from 'react-router-dom'

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
        <Router>
            <Switch>
                <Route path="/" exact>
                        <HeaderAndSideBar 
                        navBarState={navBarState} setNavBarState={setNavBarState} 
                        setClickGallery={setClickGallery} setQueueOnClick={setQueueOnClick} setStepsEnabled={setStepsEnabled}/>
                        <HomepageRightBody 
                        clickGallery={clickGallery} setClickGallery={setClickGallery} setQueueOnClick={setQueueOnClick} queueOnClick={queueOnClick} stepsEnabled={stepsEnabled} setStepsEnabled={setStepsEnabled}/>
                </Route>
                <Route path="/profile" exact>
                        <HeaderAndSideBar 
                        navBarState={navBarState} setNavBarState={setNavBarState}
                        setClickGallery={setClickGallery} setQueueOnClick={setQueueOnClick}/>
                        <Profile clickGallery={clickGallery} setClickGallery={setClickGallery} queueOnClick={queueOnClick}/>
                </Route>
            </Switch>
        </Router>
    )
}
