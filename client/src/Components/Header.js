import React,{useState,useEffect} from 'react'
import axios from 'axios'
import { useAuth0 } from '@auth0/auth0-react'

export default function Header({navBarState,setNavBarState,setStepsEnabled}) {
    const [userPicture,setUserPicture] = useState("")
    const {isAuthenticated,getAccessTokenSilently} = useAuth0()
    const heroku = 'https://schaeffler.herokuapp.com/'

    useEffect(() =>{
        async function getAccessToken(){
            if(isAuthenticated){
                const token = await getAccessTokenSilently()
                await axios.get('http://localhost:5000/protected',{//change here
                // await axios.get(heroku+'protected',{//change here
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

    const handleSupportBtn = () => {
        setStepsEnabled(true)
    }

    return (
        <header className={navBarState? "body-pd header":"header"} >
            <div className="header_toggle">
                <i className={navBarState ? "bx-x bx bx-menu":'bx bx-menu'} value={navBarState} onClick={()=> { setNavBarState(!navBarState)}}></i> 
            </div>
            <img src="schaeffler.png" alt="navLogo" className="header_img"/>

            <button className="support-btn" onClick={handleSupportBtn}>
                <img src="help.png" alt="help" className="support-icon"/>
            </button>
            
            <div className="header_profile"> 
                <img src={userPicture} alt="user"/> 

            </div>
        </header>
    )
}
