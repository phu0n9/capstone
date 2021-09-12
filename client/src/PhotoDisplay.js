import React from 'react'

export default function PhotoDisplay({showImage,setClickBiggerImg,setClickGallery}) {
    const handleCloseButton = () => {
        setClickBiggerImg(false)
        setClickGallery(true)
    }

    return (
        <div className="photo-display-wrapper"> 
            <button className="close-btn" ><img src="close.png" alt="close" className="close-icon" onClick={handleCloseButton}/></button>
            <img src={`data:image/png;base64,${showImage}`} alt="item-img-bigger" className="bigger-size"/>
        </div>
    )
}
