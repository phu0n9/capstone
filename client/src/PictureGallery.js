import React,{useCallback,useState,useRef,useEffect} from 'react'
import infinityScroll from './InfinityScroll'
import axios from 'axios'
import {useAuth0} from '@auth0/auth0-react'
import PhotoDisplay from './PhotoDisplay'

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
    const heroku = 'https://schaeffler.herokuapp.com/'

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

    const handlePictureClick = async (id) =>{
        if(isAuthenticated){
            setClicked(true)
            const token = await getAccessTokenSilently()
            // await axios.get(`http://localhost:5000/inventory/${id}`,{
                await axios.get(heroku+`inventory/${id}`,{ //change here
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
                        <button className="delete-btn">Delete</button>
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
