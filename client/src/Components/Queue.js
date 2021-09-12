import React,{useCallback,useRef} from 'react'
import popUp from '../PopUp'
import axios from 'axios'
import {useAuth0} from '@auth0/auth0-react'
require('dotenv').config()

export default function Queue({setQueueOnClick,available,setExecuteOnClick,setRackDes,setRowDes}) {
    const observer = useRef()
    const {isAuthenticated,getAccessTokenSilently} = useAuth0()

    const handleCloseButton = () => {
        setQueueOnClick(false)
    }

    const {
        queue,
        error,
        loading
    } = popUp()

    const lastItem= useCallback((node) =>{
        if(loading) return
        if(observer.current) observer.current.disconnect()
        // observer.current = new IntersectionObserver(entries =>{
        //     if(entries[0].isIntersecting && hasMore) 
        // setPageNumber(prevInventory => prevInventory + 5)
        // })
        if(node) observer.current.observe(node)
    },[loading])

    function getPosition(string, subString, index) {
        return string.split(subString, index).join(subString).length;
    }
 
    const handleExecuteItem = (async (e)=>{
        const token = await getAccessTokenSilently()
        setExecuteOnClick(true)
        if(isAuthenticated && available){
            setExecuteOnClick(false)
            const url = process.env.REACT_APP_WINDOW_LOCATION+`queue/execute/${e.target.value}` //change here
            await axios.get(url,{
                headers: {
                    authorization: `Bearer ${token}`
                }
            })
            .then((res)=>{ 
                setRowDes(res.data.substring(res.data.indexOf(".") + 1,getPosition(res.data,".",2)))
                setRackDes(res.data.substring(0,res.data.indexOf(".")))
            })
            .catch(error=> {console.log(error)})    
        }
    })

    const handleCancelItem = (async (e)=>{
        if(isAuthenticated){
            const token = await getAccessTokenSilently()
            const url = process.env.REACT_APP_WINDOW_LOCATION+`queue/delete/${e.target.value}`//change here
            await axios.delete(url,{
                headers: {
                    authorization: `Bearer ${token}`
                }
            })
            .catch(error=> {console.log(error)})    
        }
    })

    return (
        <div className="queue-wrapper">
            <button className="close-btn" onClick={handleCloseButton}><img src="close.png" alt="close" className="close-icon"/></button>
            <p className="title-text">QUEUE COMMAND SEARCH</p>
            <div className="queue-slider">
            {queue.map((item,index)=>{
                if(item.length === index +1){
                    return <div className="queue-item">
                        <p key={item.keyword} ref={lastItem} className="keyword-label">Search location: {item.keyword}</p>
                        <button type="button" className="cancel-item-btn" onClick={handleCancelItem} value={item._id} ref={lastItem}>DELETE</button>
                        <button type="button" className="execute-item-btn" onClick={handleExecuteItem}value={item._id} ref={lastItem}>EXECUTE</button>
                    </div>
                }
                else{
                    return <div className="queue-item" key={(index).toString()}>
                        <p key={(index+1).toString()} className="keyword-label">Search location: {item.keyword}</p>
                        <button key={(index+3).toString()} type="button" onClick={handleCancelItem} className="cancel-item-btn" value={item._id}>DELETE</button>
                        <button key={(index+4).toString()} type="button" onClick={handleExecuteItem} className="execute-item-btn" value={item._id}>EXECUTE</button>
                    </div>
                }
            })}
            <div>{loading && 'Loading...'}</div>
            <div>{error && 'Error'}</div>
            </div>
        </div>
    )
}

