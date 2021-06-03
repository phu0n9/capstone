import React,{useEffect,useState} from 'react'
import {io} from 'socket.io-client'
import {useParams} from 'react-router-dom'

export default function SearchBar() {
    const IP = 'localhost'
    // const IP = '172.20.10.3'
    const [socket,setSocket] = useState()
    const [clicked,setClick] = useState(false)
    const [keyword,setKeyword] = useState()
    const {id: userId} = useParams()

    useEffect(() => {
        const s = io(`http://${IP}:5000`)
        setSocket(s)
        return () =>{
            s.disconnect()  
        }
    }, []) 

    const onSearchClickButton = () =>
    {
        setClick(!clicked)
    }

    const onSearching = (event) =>{
        setKeyword(event.target.value);
    }

    useEffect(() => {
        if(socket == null) return
        
        socket.emit('get-user',userId)

    }, [socket,userId])

    useEffect(() =>{
        if (socket === null || keyword === null) return
        if(clicked === true && keyword !== null){
            socket.emit("begin-search",keyword)
            setClick(!clicked)
            setKeyword("")
        }
    },[socket,keyword,clicked])

    return (
        <form className="search-bar">
        <input
            type="text"
            placeholder="Search inventory"
            className="search-box"
            onChange={onSearching}
            value={keyword}
        />
        <button type="button" className="search-btn" onClick={onSearchClickButton}>Search</button>
    </form>
    )
}
