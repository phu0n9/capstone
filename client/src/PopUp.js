import {useState,useEffect} from 'react'
import axios from 'axios'
import Pusher from 'pusher-js'

export default function PopUp() {

    const [queue,setQueue] = useState([])
    const [error,setError] = useState(false)
    const heroku = 'https://schaeffler.herokuapp.com/queue'
    // 'http://localhost:5000/queue'
    function getQueue(){
        setError(false)
        axios({
            method:'GET',
            url: heroku,
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
        const pusher = new Pusher('2ccb32686bdc0f96f50a',{
            'cluster':'ap1',
            encrypted:true
        })
        const channel = pusher.subscribe('tasks')
        channel.bind('deleted',function(){
            fetchChanges()
        })
        channel.bind('inserted',function(){
            fetchChanges()
        })
        return () => channel.unbind()
    },[])

    return {queue,error}
}
