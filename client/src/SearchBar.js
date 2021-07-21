import React,{useEffect,useState,useCallback,useRef} from 'react'
import {io} from 'socket.io-client'
import popUp from './PopUp'
import axios from 'axios'


export default function SearchBar({userId}) {
  
    const [socket,setSocket] = useState()
    const [pressed,setPress] = useState(false)
    const [keyword,setKeyword] = useState()
    const [buttonClicked,setButtonClicked] = useState(false)
    const [enablePopUp,setEnablePopUp] = useState(false)
    const [cancelBtn,setCancelBtn] = useState(false)
    const observer = useRef()

     // const [clicked,setClicked] = useState(false)
    const heroku = 'https://schaeffler.herokuapp.com/'
    const {
        queue,
        error,
    } = popUp()

    const lastItem= useCallback((node) =>{
        if(observer.current) observer.current.disconnect()
        observer.current = new IntersectionObserver(entries =>{
            if(entries[0].isIntersecting) 
            console.log("here")
        })
        if(node) observer.current.observe(node)
    },[])

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
        // setClicked(true)
        setButtonClicked(true)
    }

    const onSearching = (event) =>{
        setKeyword(event.target.value);
    }

    useEffect(() =>{
        if (socket === null || keyword === null) return

        if((pressed === true ) || (buttonClicked === true)){
            if (keyword !== ""){
                const key = {
                    "keyword":keyword,
                    "userId":userId,
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
    },[socket,keyword,pressed,userId,buttonClicked])
    
    useEffect(() =>{
        if(socket == null) return
        const handler = (delta) =>{
            setEnablePopUp(delta)
        }
        socket.on('popup',handler)

        return () =>{
            socket.off('popup',handler)
        }
    },[socket,enablePopUp])

    const handleCancelButton = (()=>{
        setCancelBtn(true)
    })

    const handleExecuteItem = ((e)=>{
        const url = `http://localhost:5000/queue/execute/${e.target.value}`
        axios.get(url)
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
                        if(item.length === index +1){
                            return <div className="queue-item">
                                <span key={item.keyword} ref={lastItem} >Search location: {item.keyword}</span>
                                <button type="button" className="cancel-item-btn" onClick={handleCancelItem} value={item._id} ref={lastItem}>cancel</button>
                                <button type="button" className="execute-item-btn" onClick={handleExecuteItem} value={item._id} ref={lastItem}>execute</button>
                            </div>
                        }
                        else{
                            return <div className="queue-item">
                                <span key={item.keyword}>Search location: {item.keyword}</span>
                                <button type="button" className="cancel-item-btn" onClick={handleCancelItem} value={item._id}>cancel</button>
                                <button type="button" className="execute-item-btn" onClick={handleExecuteItem} value={item._id}>execute</button>
                            </div>
                        }
                    })}
                    <div>{error && 'Error'}</div>
                </div>

            </div> : ""}

        </>
       
    )
}
