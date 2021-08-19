const {Schema,model} = require('mongoose')

const Process = new Schema({       
    name:{
        type:String,
        required:true,
        default:false
    },
    onProcess:{
        type:Boolean,
        required:true
    }
},{   
    timestamps: true,
})

module.exports = model("Process",Process)