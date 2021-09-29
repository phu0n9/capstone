const {Schema,model} = require('mongoose')

const Inventory = new Schema({
    location:{
        type:String,
        required:true
    },
    photo:{
        type:Object,
        required:true
    },
    userId:{
        type:String,
        required:true
    }
},{   
    timestamps: true,
})

module.exports = model("Inventory",Inventory)