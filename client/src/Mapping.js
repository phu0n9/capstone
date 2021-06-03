import {useEffect,useState} from 'react'
import {io} from 'socket.io-client'
import {useParams} from 'react-router-dom'

export default function Mapping({setPhoto,setStatus}) {
    // const [inventory, setInventory] = useState()
    // const [file,setFile] = useState()
    const [socket,setSocket] = useState()
    // const [state,setBtn] = useState(false)
    const {id: userId} = useParams()
    // const [map,setMap] = useState(new Map())

    // const IP = '172.20.10.3'
    const IP = 'localhost'

    useEffect(() => {
        const s = io(`http://${IP}:5000`)
        setSocket(s)
        return () =>{
            s.disconnect()  
        }
    }, [])  

    // const handleChange = (event) => {
    //     setInventory(event.target.value);
    // }

    // const handleFile = (event) =>{
    //     setFile(event.target.value)
    //     const reader = new FileReader();
    //     const file = event.target.files[0];
    //     reader.onloadend = () => {
    //         setPhoto(reader.result);
    //     };
    //     reader.readAsDataURL(file);
    // }

    // useEffect(()=>{
    //     if (socket == null || inventory == null || file == null || map == null) return
    //     if(state === true && inventory !== null && file !== null){
    //         map[String(inventory)] = file
    //         socket.emit("send-changes",map)
    //         setMap(map)
    //         setBtn(!state)
    //         setInventory("")
    //         setFile("")
    //         setStatus(Object.keys(map)+",")
    //     }
    // },[inventory,socket,state,file,map])

    // const handleInventory = () =>{
    //     // map[String(inventory)] = file
    //     setBtn(!state)
    //     socket.emit('save-inventory',map)
    //     setStatus(Object.keys(map))
    // }

    // useEffect(() =>{
    //     if(socket == null) return
    //     const handler = (delta) =>{
    //         // setStatus(String(Object.keys(delta)))
    //         // const reader = new FileReader()
    //         // const file = Object.values(delta)
    //         // reader.onloadend = () => {
    //         //     setPhoto(reader.result)
    //         // }
    //         // reader.readAsDataURL(file)
    //         setMap(delta)
    //         console.log("this = ",delta)
    //     }
    //     socket.on('receive-changes',handler)
    //     setStatus(Object.keys(map)+",")
    //     return () =>{
    //         socket.off('receive-changes',handler)
    //     }
    // },[socket,map])

    useEffect(() => {
        if(socket == null) return
        
        socket.emit('get-user',userId)

    }, [socket,userId])


//TODO: broadcast changes with the stack in the side bar
//  and communication with Raspberry Pi
 
    return (
        <div className="mapping-wrapper">
            {/* <label htmlFor="inventoryName">Inventory adress:</label><br></br>
            <input type="text" value={inventory} placeholder="Inventory" onChange={handleChange}/><br></br>
            <input type="file" name="picture" value={file} onChange={handleFile}/><br></br><span></span>
            <input type="submit" placeholder="Submit" value="submit" onClick={handleInventory}/>
            <div>{inventory}</div>  */}
            Mapping
        </div> 
    )
}

