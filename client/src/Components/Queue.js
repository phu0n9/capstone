import React from 'react'

export default function Queue({setQueueOnClick}) {
    return (
        <button onClick={() => setQueueOnClick(true)}>Queue pop up</button>
    )
}

