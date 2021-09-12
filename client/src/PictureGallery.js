import React,{useCallback,useState,useRef,useEffect} from 'react'
import infinityScroll from './InfinityScroll'
import axios from 'axios'
import {useAuth0} from '@auth0/auth0-react'
import PhotoDisplay from './PhotoDisplay'
import Pusher from 'pusher-js'
require('dotenv').config()

export default function PictureGallery({setClickGallery,setRowClick,setRackClick,keyword,setKeyword}) {
    const observer = useRef()
    const [pageNumber,setPageNumber] = useState(5)
    const [itemLocation,setItemLocation] = useState("")
    const [createAt,setCreateAt] = useState("")
    const [updateAt,setUpdateAt] = useState("")
    const {isAuthenticated,getAccessTokenSilently} = useAuth0()
    const [clicked,setClicked] = useState(false)
    const [showImage,setShowImage] = useState("")
    const [clickBiggerImg,setClickBiggerImg] = useState(false)
    const [itemId,setItemId] = useState(undefined)

    const {
        inventory,
        hasMore,
        error,
        loading
    } = infinityScroll(pageNumber,keyword)
    

    const lastInventory= useCallback((node) =>{
        if(loading) return
        if(observer.current) observer.current.disconnect()
        observer.current = new IntersectionObserver(entries =>{
            if(entries[0].isIntersecting && hasMore) setPageNumber(prevInventory => prevInventory + 5)
        })
        if(node) observer.current.observe(node)
    },[loading,hasMore])

    useEffect(() =>{
        const pusher = new Pusher(process.env.REACT_APP_PUSHER_KEY,{
            'cluster':process.env.REACT_APP_PUSHER_CLUSTER,
            forceTLS: true,
        })
        const channel = pusher.subscribe('tasks')
        channel.bind('inserted',function(){
            setPageNumber(5)
        })
        channel.bind('itemDeleted',function(){
            setPageNumber(5)
            setClicked(false)
        })
    },[])

    const handlePictureClick = async (id) =>{
        if(isAuthenticated){
            setClicked(true)
            setItemId(id)
            const token = await getAccessTokenSilently()
                await axios.get(process.env.REACT_APP_WINDOW_LOCATION+`inventory/${id}`,{ 
                headers:{
                    authorization: `Bearer ${token}`
                }
            })
            .then(res =>{
                setItemLocation(res.data.location)
                setCreateAt(res.data.createdAt)
                setUpdateAt(res.data.updatedAt)
                setShowImage(res.data.photo)
            })
            .catch(err => console.log(err))

        }
    }

    const handleCloseButton = () => {
        setClickGallery(false)
        setClicked(false)
        setRowClick(undefined)
        setRackClick(undefined)
        setKeyword(undefined)
    }

    const handleBiggerImgClick = () => {
        setClickBiggerImg(true)
    }

    const handleDeleteItem = async (e) =>{
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
    }

    return ( 
        <> 
            {clickBiggerImg ? <PhotoDisplay showImage={showImage} setClickBiggerImg ={setClickBiggerImg} setClickGallery={setClickGallery}/>
        : <div className="gallery-wrapper">
            <button className="close-btn" onClick={handleCloseButton}><img src="close.png" alt="close" className="close-icon"/></button>
            <p className="title-text">PICTURE GALLERY</p>
            <div className="picture-slider">
                {inventory.map((item,index)=>{
                    if(inventory.length === index +1){
                        return <img key={index} ref={lastInventory} src={`data:image/png;base64,${item.photo}`} alt="sideBarPhoto" className="img-slider" onClick={() => handlePictureClick(item._id)}/>
                    }
                    else{
                        return <img key={index} src={`data:image/png;base64,${item.photo}`} alt="sideBarPhoto" className="img-slider" onClick={() => handlePictureClick(item._id)}/> 
                    }
                })}
                <div>{loading && 'Loading...'}</div>
                <div>{error && 'Error'}</div>
            </div>
            {clicked ?  <div className="picture-info">
                <div className="item-info-wrapper">
                    <div className="gallery-btn">
                        <button className="delete-btn" onClick={handleDeleteItem} value={itemId}>Delete</button>
                    </div> <br />
                    <div className="item-info">
                        <p>Item location: {itemLocation}</p>
                        <p>Created At: {createAt}</p>
                        <p>Updated At: {updateAt}</p>
                    </div>
                </div>
                    <span>
                        <img src={`data:image/png;base64,${showImage}`} alt="item-img" className="item-picture" onClick={handleBiggerImgClick}/>
                    </span>
            </div>: ""}
        </div>
        }
    </>
        

       
    )
}
