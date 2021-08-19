import React from 'react'

export default function Upload({setUploadOnClick}) {
    return (
        <button onClick={() => setUploadOnClick(true)}>Upload csv file</button>
    )
}
