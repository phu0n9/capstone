import React from 'react'

export default function PhotoDisplay({clickPhoto,setOnClick,onClick}) {
    const photoDisplayOnClick = (()=>{
        setOnClick(true)
    })
    return (
        <div className="photo-display-wrapper" onClick={photoDisplayOnClick} style={onClick ? {height: 455}: {height: 300}}>
            <img src={clickPhoto} alt="" className={onClick ? "photo-display":"minimized-photo-display"}/>
        </div>
    )
}
