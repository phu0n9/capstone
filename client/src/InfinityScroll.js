import {useEffect,useState} from 'react'
import axios from 'axios'
import Pusher from 'pusher-js'
require('dotenv').config()

export default function InfinityScroll(pageNumber,keyword,selection) {
    const [loading,setLoading] = useState(true)
    const [error,setError] = useState(false)
    const [inventory,setInventory] = useState([])
    const [hasMore,setHasMore] = useState(false)       
    const [change,setChange] = useState(false)
    const heroku = 'https://schaeffler.herokuapp.com/'

    function fetchApi(URL,paramsDict){
        axios({
            method:'GET',
            url: URL,
            params:paramsDict
        })
        .then(res => {
            setInventory(() =>{
                return [...new Set([...'',...res.data.map(i => i)])]
            })
            // setHasMore(res.data.length > 0)
            // setLoading(false)
        })
        .catch(e =>{
            setError(true)
            console.log('Error: '+e)
        })
    }
    
    // 'http://localhost:5000/inventory'
    useEffect(() =>{
        const pusher = new Pusher(process.env.PUSHER_KEY,{
            'cluster':process.env.PUSHER_CLUSTER,
            encrypted:true
        })
        const channel = pusher.subscribe('tasks')
        channel.bind('inserted',function(){
            console.log("updated")
            fetchApi(heroku+'inventory',{page:5})
            setChange(true)
        })
        return () => channel.unbind('inserted')
    },[])


    useEffect(() =>{
        if(keyword !== undefined){
            switch(selection){
                case "location":
                    fetchApi(heroku+'sort/sortByLocation',{location:keyword})
                    break
                case "userId":
                    fetchApi(heroku+'sort/sortByUserId',{userId:keyword})
                    break
                case "date":
                    fetchApi(heroku+'sort/sortByTime',{time:keyword})
                    break
                default:
                    break
            }
        }
    },[keyword,selection])

    useEffect(() =>{
        setLoading(true)
        setError(false)
        if(keyword === ''){
            axios({
                method:'GET',
                url: heroku+'inventory',
                params:{page:pageNumber}
            })
            .then(res => {
                setInventory(prevInventory =>{
                    return [...new Set([...prevInventory,...res.data.map(i => i)])]
                })
                    setHasMore(res.data.length > 0)
                    setLoading(false)
                    setChange(false)
            })
            .catch(e =>{
                setError(true)
                console.log('Error: '+e)
            })
        }
    },[pageNumber,keyword])

    return {loading,hasMore,error,inventory,change}
}
