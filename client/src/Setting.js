import React,{useState,useEffect} from 'react'
import LogoutButton from './Components/LogoutButton'
import Upload from './Components/Upload'
import Queue from './Components/Queue'
import {useAuth0} from '@auth0/auth0-react'
import axios from 'axios'

export default function Setting({setQueueOnClick,setUploadOnClick}) {
    const [userPicture,setUserPicture] = useState("")
    const {isAuthenticated,getAccessTokenSilently} = useAuth0()
    const heroku = 'https://schaeffler.herokuapp.com/'

    useEffect(() =>{
        async function getAccessToken(){
            if(isAuthenticated){
                const token = await getAccessTokenSilently()
                // await axios.get('http://localhost:5000/protected',{//change here
                await axios.get(heroku+'protected',{//change here

                    headers: {
                        authorization:`Bearer ${token}`
                    }
                })
                .then(response => {
                    setUserPicture(response.data.picture)
                })
                .catch(err => console.log(err))
            }
        }
        getAccessToken()  
    },[getAccessTokenSilently,isAuthenticated])

    return (
        <div className="dropdown">
            <img src={userPicture} alt="user" className="img-wrapper"/>
            <div className="dropdown-content">
                <Upload setUploadOnClick={setUploadOnClick}/>
                <Queue setQueueOnClick={setQueueOnClick}/>
                <LogoutButton/>
            </div>
        </div>
    )
}
