import React,{useEffect,useState} from 'react'
import {io} from 'socket.io-client'
import popUp from './PopUp'
import axios from 'axios'


export default function SearchBar({userId}) {
  
    const [socket,setSocket] = useState()
    const [pressed,setPress] = useState(false)
    const [keyword,setKeyword] = useState("")
    const [buttonClicked,setButtonClicked] = useState(false)
    const [enablePopUp,setEnablePopUp] = useState(false)
    const [cancelBtn,setCancelBtn] = useState(false)
    const [enableSearching,setEnableSearching] = useState(false)
    const [enableLanding,setEnableLanding] = useState(false)
    const heroku = 'https://schaeffler.herokuapp.com/'
    const {
        queue,
        error,
        landingStatus,searchStatus
    } = popUp()

    // 'http://localhost:5000/'
    useEffect(() => {
        const s = io('http://localhost:5000/')
        setSocket(s)
        return () =>{
            s.disconnect()  
        }
    }, []) 

    const onPress = (event) =>
    {
        if(event.key === 'Enter'){
            setPress(true)
        }
    }

    const onSearchClickButton = () =>
    {
        setButtonClicked(true)
    }

    const onSearching = (event) =>{
        setKeyword(event.target.value);
    }

    useEffect(() =>{
        if (socket === null) return

        if((pressed === true ) || (buttonClicked === true)){
            if (keyword !== ""){
                const key = {
                    "keyword":keyword,
                    // "userId":userId,
                    "socketId":socket.id
                }
                socket.emit("begin-search",key)
                setPress(false)
                setButtonClicked(false)
                setKeyword("")
            }
            else{
                alert("Please enter a keyword")
            } 
        }
    },[socket,keyword,pressed,buttonClicked])
    
    useEffect(() =>{
        if(socket == null) return
        const handler = (delta) =>{
            setEnablePopUp(delta)
            setCancelBtn(false)
        }
        socket.on('popup',handler)

        return () =>{
            socket.off('popup',handler)
        }
    },[socket,enablePopUp])

    useEffect(() =>{
        if(searchStatus === "available"){
            setEnableSearching(false)
        }
        else if(searchStatus === "searching"){
            setEnableSearching(true)
        }
        else{
            setEnableSearching(false)
        }
    },[searchStatus])

    useEffect(() =>{
        if(landingStatus === "up"){
            setEnableLanding(false)
        }
        else if(landingStatus === "down"){
            setEnableLanding(true)
        }
        else{
            setEnableLanding(false)
        }
    },[landingStatus])

    useEffect(() =>{
        if(enableSearching){
            setEnablePopUp(false)
            setEnableLanding(false)
        }
        else if(enableLanding){
            setEnablePopUp(false)
            setEnableSearching(false)
        }
    },[enableSearching,enableLanding])

    const handleCancelButton = (()=>{
        setCancelBtn(true)
        setEnablePopUp(false)
    })

    const handleExecuteItem = ((e)=>{
        const url = `http://localhost:5000/queue/execute/${e.target.value}`
        axios.get(url)
        .then(()=>{ setEnableSearching(true)})
        .catch(error=> {console.log(error)})    
    })

    const handleCancelItem = ((e)=>{
        const url = `http://localhost:5000/queue/delete/${e.target.value}`
        axios.delete(url)
        .catch(error=> {console.log(error)})    
    })

    return (
        <>
            <span >
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
