import React from 'react'
// import {io} from 'socket.io-client'

export default function Mapping({onClick,setOnClick}) {
    // const [socket,setSocket] = useState()

    // useEffect(() => {
    //     const s = io(`http://localhost:5000`)
    //     setSocket(s)
    //     return () =>{
    //         s.disconnect()  
    //     }
    // }, [])  

    const mappingOnClick = (()=>{
        setOnClick(false)
    })
 
    return (
        <div  className="mapping-wrapper" onClick={mappingOnClick} style={onClick ? {height: 300}: {height: 455}} >
          
            Mapping
        </div> 
    )
}

