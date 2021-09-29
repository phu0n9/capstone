const {Schema,model} = require('mongoose')

const Queue = new Schema({
    keyword:{
        type:String,
        required:true
    },
    userId:{
        type:String,
        required:true
    }
},{   
    timestamps: true,
})

module.exports = model("Queue",Queue)