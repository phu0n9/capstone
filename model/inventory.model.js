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
        required:true,
        type:String
    }
},{   
    timestamps: true,
})

module.exports = model("Inventory",Inventory)