import React,{useEffect,useState} from 'react'
import {io} from 'socket.io-client'
import popUp from './PopUp'
import axios from 'axios'
import {useAuth0} from '@auth0/auth0-react'

export default function SearchBar({queueOnClick,setQueueOnClick,setUploadOnClick,uploadOnClick}) {
    const [socket,setSocket] = useState()
    const [pressed,setPress] = useState(false)
    const [keyword,setKeyword] = useState("")
    const [buttonClicked,setButtonClicked] = useState(false)
    const [enablePopUp,setEnablePopUp] = useState(false)
    const [cancelBtn,setCancelBtn] = useState(false)
    const [enableSearching,setEnableSearching] = useState(false)
    const [enableLanding,setEnableLanding] = useState(false)
    const [userId,setUserId] = useState("")
    const [enableUpload,setEnableUpload] = useState(uploadOnClick)
    const heroku = 'https://schaeffler.herokuapp.com/'
    const {
        queue,
        error,
        landingStatus,searchStatus
    } = popUp()

    const {isAuthenticated,getAccessTokenSilently} = useAuth0()

    // 'http://localhost:5000/'
    useEffect(() => {
        // const s = io('http://localhost:5000/') //change here
        const s = io(heroku) //change here
        setSocket(s)
        return () =>{
            s.disconnect()
        }
    }, []) 

    const onPress = (event) =>{
        if(event.key === 'Enter'){
            setPress(true)
        }
    }

    const onSearchClickButton = () => setButtonClicked(true)
    const onSearching = (event) =>setKeyword(event.target.value)

    useEffect(()=>{
        if(queueOnClick){
            setEnablePopUp(true)
            setCancelBtn(false)
        }
        else{
            setEnablePopUp(false)
            setCancelBtn(true)
        }
    },[queueOnClick])

    useEffect(() => {
        if(uploadOnClick){
            setCancelBtn(false)
            setEnableUpload(true)
            setEnablePopUp(false)
        }
        else{
            setCancelBtn(true)
            setEnableUpload(false)
        }
    },[uploadOnClick])

    useEffect(() =>{
        async function getAccessToken(){
            if(isAuthenticated){
                const token = await getAccessTokenSilently()
                // await axios.get('http://localhost:5000/protected',{ //change here
                await axios.get(heroku+'protected',{ //change here
                    headers: {
                        authorization:`Bearer ${token}`
                    }
                })
                .then(response => {
                    setUserId(response.data.sub)
                })
                .catch(err => console.log(err))
            }
        }
        getAccessToken()  
    },[getAccessTokenSilently,isAuthenticated])

    useEffect(() =>{
        if (socket === null) return      
        var regexp = /^\d{1,2}\.\d{1,2}\.\d{1,2}?$/
        if((pressed === true ) || (buttonClicked === true)){
            if (keyword !== "" && regexp.test(keyword)){
                const key = {
                    "keyword":keyword,
                    "userId":userId
                }
                socket.emit("begin-search",key)
                setPress(false)
                setButtonClicked(false)
                setKeyword("")
            }
            else if(!regexp.test(keyword)){
                alert("Invalid keyword")
            }
            else{
                alert("Please enter a keyword")
            } 
        }
    },[socket,keyword,pressed,buttonClicked,userId])
    
    useEffect(() =>{
        if(socket == null) return
        const handler = (delta) =>{
            setEnablePopUp(delta)
            setCancelBtn(false)
        }
        socket.on('popup',handler)
        return () => socket.off('popup',handler)
        
    },[socket,enablePopUp])

    useEffect(() =>{
        setEnableSearching((searchStatus === "searching") ? true : false)
        setEnableLanding((landingStatus === "down") ? true: false)

    },[searchStatus,landingStatus])

    useEffect(() =>{
        if(enableSearching){
            setEnablePopUp(false)
            setQueueOnClick(false)
            setEnableLanding(false)
            setEnableUpload(false)
            setUploadOnClick(false)
        }
        else if(enableLanding){
            setEnablePopUp(false)
            setQueueOnClick(false)
            setEnableSearching(false)
            setEnableUpload(false)
            setUploadOnClick(false)
        }
    },[enableSearching,enableLanding,setQueueOnClick,setUploadOnClick])

    const handleCancelButton = (()=>{
        setCancelBtn(true)
        setEnablePopUp(false)
        setQueueOnClick(false)
        setUploadOnClick(false)
    })

    const handleExecuteItem = (async (e)=>{
        if(isAuthenticated){
            const token = await getAccessTokenSilently()
            // const url = `http://localhost:5000/queue/execute/${e.target.value}` //change here
            const url = heroku+`queue/execute/${e.target.value}` //change here
            axios.get(url,{
                headers: {
                    authorization: `Bearer ${token}`
                }
            })
            .then(()=>{ 
                // setEnableSearching(true)
                socket.emit('execute',"default")
            })
            .catch(error=> {console.log(error)})    
        }
    })

    useEffect(()=>{
        if(socket == null) return
        const handler = (delta) =>{
            setEnableSearching(delta)
        }
        socket.once('executePopUp',handler)
        return () => socket.off('executePopUp',handler)
    },[socket])

    const handleCancelItem = (async (e)=>{
        if(isAuthenticated){
            const token = await getAccessTokenSilently()
            // const url = `http://localhost:5000/queue/delete/${e.target.value}`//change here
            const url = heroku+`queue/delete/${e.target.value}`//change here
            axios.delete(url,{
                headers: {
                    authorization: `Bearer ${token}`
                }
            })
            .catch(error=> {console.log(error)})    
        }
    })

    return (
        <>
            <span>
                <input
                    type="search"
                    placeholder="Search inventory"
                    className="search-box"
                    onChange={onSearching}
                    value={keyword}
                    onKeyPress={onPress}
                />
                <button type="button" className="search-btn" onClick={onSearchClickButton}>          
                    <img src="search.png" alt="manifying-glass" />
                </button>
            </span>

            {enablePopUp ?  <div className={cancelBtn ? "popup-hidden": "popup-container"}>
            <button className="cancel-button" onClick={handleCancelButton}>X</button>     
                <div className="queue-container">
                    {queue.map((item,index)=>{
                        // if(item.length === index +1){
                        //     return <div className="queue-item">
                        //         <span key={item.keyword} ref={lastItem} >Search location: {item.keyword}</span>
                        //         <button type="button" className="cancel-item-btn" onClick={handleCancelItem} value={item._id} ref={lastItem}>cancel</button>
                        //         <button type="button" className="execute-item-btn" onClick={handleExecuteItem} value={item._id} ref={lastItem}>execute</button>
                        //     </div>
                        // }
                        // else{
                            return <div className="queue-item" key={(index).toString()}>
                                <span key={(index+1).toString()}>Search location: {item.keyword}</span>
                                <button key={(index+2).toString()} type="button" className="cancel-item-btn" onClick={handleCancelItem} value={item._id}>cancel</button>
                                <button key={(index+3).toString()} type="button" className="execute-item-btn" onClick={handleExecuteItem} value={item._id}>execute</button>
                            </div>
                        // }
                    })}
                    <div>{error && 'Error'}</div>
                </div>

            </div> : ""}
            {enableUpload ? <div className={cancelBtn ? "popup-hidden": "popup-container"}>
                <button className="cancel-button" onClick={handleCancelButton}>X</button>     
                <input type="file" className="upload-item"/>
            </div> : ""}
            {enableSearching ?
            <div className="popup-container">
                <img className="search-gif" src="search-gif.gif" alt="search-gif" />
                <p className="search-text">Searching for item</p>
            </div> 
            : ""}
            {enableLanding ? <div className="popup-container">{landingStatus}</div>: ""}
            
            
        </>
       
    )
}
