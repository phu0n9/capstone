import React,{useEffect,useState} from 'react'
import {io} from 'socket.io-client'

export default function SidebarBox({setClickPhoto}) {
    const [location,setLocation] = useState()
    const [photo,setPhoto] = useState()
    const [socket,setSocket] = useState()

    useEffect(() => {
        const s = io(`https://schaeffler.herokuapp.com`)
        setSocket(s)
        return () =>{
            s.disconnect()  
        }
    }, [])  

    useEffect(() =>{
        if (socket == null) return
        const handler = (delta) =>{
            setLocation(delta['location'])
            setPhoto(delta['photo'])
            socket.emit('save-data',delta)
        }
        socket.on('display-result',handler)
        return () =>{
            socket.off('display-result',handler)
        }
    },[socket,photo,location])


    const photoOnClick = (() => {
        setClickPhoto(photo)
    })

    return (
        <div className="grid-item sidebar-wrapper">
            <div className="inventory-item">
            <div>Location : {location}</div>
            <img src={photo} alt="sideBarPhoto" onClick={photoOnClick} className="img-sideBar"/>
            </div>
            
        </div>
    )
}
