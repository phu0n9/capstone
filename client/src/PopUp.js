import {useState,useEffect} from 'react'
import axios from 'axios'
import Pusher from 'pusher-js'
import {useAuth0} from '@auth0/auth0-react'

require('dotenv').config()

export default function PopUp() {
    const [loading,setLoading] = useState(true)
    const [hasMore,setHasMore] = useState(false)       
    const [queue,setQueue] = useState([])
    const [error,setError] = useState(false)
    const {isAuthenticated,getAccessTokenSilently} = useAuth0()
    
    useEffect(() =>{
        async function getQueue(){
            if(isAuthenticated){
                const token = await getAccessTokenSilently()
                setError(false)
                await axios({
                    method:'GET',
                    url: process.env.REACT_APP_WINDOW_LOCATION+"queue",
                    headers: {
                        authorization: `Bearer ${token}`
                    }
                })
                .then(res => {
                    setQueue(prevInventory =>{
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
        }
        setLoading(true)
        setError(false)
        getQueue()
    },[getAccessTokenSilently,isAuthenticated])

    useEffect(() =>{
        async function fetchChanges(){
            if(isAuthenticated){
                const token = await getAccessTokenSilently()
                setError(false)
                axios({
                    method:'GET',
                    url: process.env.REACT_APP_WINDOW_LOCATION+"queue",
                    headers: {
                        authorization: `Bearer ${token}`
                    }
                })
                .then(res => {
                    setQueue(() =>{
                        return [...new Set([...'',...res.data.map(i => i)])]
                    })
                })
                .catch(e =>{
                    setError(true)
                    console.log('Error: '+e)
                })
            }
        }
        const pusher = new Pusher(process.env.REACT_APP_PUSHER_KEY,{
            'cluster':process.env.REACT_APP_PUSHER_CLUSTER,
            forceTLS: true,
        })
        const channelTask = pusher.subscribe('tasks')
        channelTask.bind('deleted',function(){
            fetchChanges()
        })
        channelTask.bind('inserted',function(){
            fetchChanges()
        })
        return () => {
            pusher.unsubscribe(channelTask)
        }
    },[getAccessTokenSilently,isAuthenticated])

    return {queue,error,hasMore,loading}
}
