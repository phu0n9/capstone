import {useEffect,useState} from 'react'
import axios from 'axios'
import Pusher from 'pusher-js'

export default function InfinityScroll(pageNumber) {
    const [loading,setLoading] = useState(true)
    const [error,setError] = useState(false)
    const [inventory,setInventory] = useState([])
    const [hasMore,setHasMore] = useState(false)       
    const [change,setChange] = useState(false)
    
    useEffect(() =>{
        const pusher = new Pusher('2ccb32686bdc0f96f50a',{
            'cluster':'ap1',
            encrypted:true
        })
        const channel = pusher.subscribe('tasks')
        channel.bind('inserted',function(){
            // setChange(true)
            setLoading(true)
            setError(false)
            axios({
                method:'GET',
                url: 'http://localhost:5000/inventory',
                params:{page:5}
            })
            .then(res => {
                setInventory(() =>{
                    return [...new Set([...inventory,...res.data.map(i => i)])]
                })
                setHasMore(res.data.length > 0)
                setLoading(false)
            })
            .catch(e =>{
                setError(true)
                console.log('Error: '+e)
            })
        })
        return () => channel.unbind('inserted')
    },[inventory])

    const refresh = () =>{
        setLoading(true)
        setError(false)
        // setChange(false)
        axios({
            method:'GET',
            url: 'http://localhost:5000/inventory',
            params:{page:pageNumber}
        })
        .then(res => {
            setInventory(prevInventory =>{
                return [...new Set([...prevInventory,...res.data.map(i => i)])]
            })
            setHasMore(res.data.length > 0)
            setLoading(false)
        })
        .catch(e =>{
            setError(true)
            console.log('Error: '+e)
        })
    }

    useEffect((refresh),[pageNumber])

    return {loading,hasMore,error,inventory,change}
}
