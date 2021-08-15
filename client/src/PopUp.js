import {useState,useEffect} from 'react'
import axios from 'axios'
import Pusher from 'pusher-js'
require('dotenv').config()

export default function PopUp() {

    const [queue,setQueue] = useState([])
    const [error,setError] = useState(false)
    const [landingStatus,setLandingStatus] = useState("up")
    const [searchStatus, setSearchStatus] = useState("available")
    const heroku = 'https://schaeffler.herokuapp.com/queue'
    
    // 'http://localhost:5000/queue'
    function getQueue(){
        setError(false)
        axios({
            method:'GET',
            url: 'http://localhost:5000/queue',
        })
        .then(res => {
            setQueue(prevInventory =>{
                return [...new Set([...prevInventory,...res.data.map(i => i)])]
            })
        })
        .catch(e =>{
            setError(true)
            console.log('Error: '+e)
        })
    }
    useEffect(() => getQueue(),[])

    function fetchChanges(){
        setError(false)
            axios({
                method:'GET',
                url: heroku,
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

    useEffect(() =>{
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
        const channelLanding = pusher.subscribe('armarker')
        channelLanding.bind('land',function(data){
            setLandingStatus(data.landingStatus)
        })
        const channelSearch = pusher.subscribe('search')
        channelSearch.bind('keyword',function(data){
            if (data !== ""){
                setSearchStatus("searching")
            }
            else setSearchStatus("done")
        })
        return () => {
            channelTask.unbind()
            channelLanding.unbind()
            channelSearch.unbind()
        }
    },[])

    return {queue,error,landingStatus,searchStatus}
}
