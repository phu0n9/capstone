import React,{useEffect,useState} from 'react'
import axios from 'axios'

export default function InfinityScroll(pageNumber) {
    const [loading,setLoading] = useState(true)
    const [error,setError] = useState(false)
    const [inventory,setInventory] = useState([])
    const [hasMore,setHasMore] = useState(false)

    useEffect(() =>{
        setLoading(true)
        setError(false)

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
    },[pageNumber])
    return {loading,hasMore,error,inventory}
}
