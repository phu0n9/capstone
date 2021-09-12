import React,{useEffect,useState} from 'react'
import axios from 'axios'
import {useAuth0} from '@auth0/auth0-react'

export default function SearchBar({setDeviceCheck,deviceCheck,setButtonSubmit,buttonSubmit,setQueueOnClick}) {
    const {user,isAuthenticated,getAccessTokenSilently} = useAuth0()
    const [keyword,setKeyword] = useState("")
    const [userId,setUserId] = useState(user.sub)
    require('dotenv').config()

    const onSearchClickButton = () => {
        if(!deviceCheck){
            alert('Please make sure to choose the device.')
        }
        else {
            setButtonSubmit(true)
            setDeviceCheck(false)
            setUserId(user.sub)
        }
    }

    const onSearching = (event) =>setKeyword(event.target.value)

    useEffect(() =>{
       
        async function getAccessToken(keyword,userId){
            if(isAuthenticated){
                const token = await getAccessTokenSilently()
                const headers = {
                    'Content-Type': 'application/json',
                    'Authorization': `Bearer ${token}`
                }
                await axios.post(process.env.REACT_APP_WINDOW_LOCATION+'search' //change here
                ,{keyword:keyword,userId:userId},{headers:headers}
                ).then(() => {})
                .catch(err => console.log(err))
            }
        }

        var regexp = /^\d{1,2}\.\d{1,2}\.\d{1,2}?$/
        if(buttonSubmit === true){
            if (keyword !== "" && regexp.test(keyword)){
                getAccessToken(keyword,userId)
                setButtonSubmit(false)
                setQueueOnClick(true)
                setKeyword("")
                setUserId("")
            }
            else if(!regexp.test(keyword)){
                alert("Invalid keyword")
            }
            else{
                alert("Please enter a keyword")
            } 
        }
    },[keyword,buttonSubmit,userId,setButtonSubmit,getAccessTokenSilently,isAuthenticated,setQueueOnClick])

    return (
        <>
            <p className="title-text">SEARCH TOOL</p>
            <input
                type="search"
                placeholder="Search inventory"
                className="search-box"
                onChange={onSearching}
                value={keyword}
            />
            <img src="search.png" alt="manifying-glass" className="search-btn"/>
            <div className="radio-btn-wrapper"></div>
            <button type="submit" className="search-btn" onClick={onSearchClickButton}>SEARCH</button>
        </>
       
    )
}
