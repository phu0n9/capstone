const {Schema,model} = require('mongoose')
const bcrypt = require('bcrypt-nodejs')
const SALT_WORK_FACTOR = 10

const User = new Schema({
    username:{
        type:String,
        required:true,
        unique:true
    },
    password:{
        type:String,
        required:true,
        minlength:3
    },
    // searchData: String,
    // inventoryId: [{type: Schema.Types.ObjectId, ref:'Inventory' }]
},{   
    timestamps: true,
})

User.pre('save',function(next){
    var user = this

    // only hash the password if it has been modified (or is new)
    if (!user.isModified('password')) return next();

    // generate a salt
    bcrypt.genSalt(SALT_WORK_FACTOR, function(err, salt) {
        if (err) return next(err)

        // hash the password using our new salt
        bcrypt.hash(user.password, salt, function(err, hash) {
            if (err) return next(err)
            // override the cleartext password with the hashed one
            user.password = hash
            next()
        })
    })
})

User.methods.comparePassword = function(candidatePassword, cb) {
    bcrypt.compare(candidatePassword, this.password, function(err, isMatch) {
        if (err) return cb(err)
        cb(null, isMatch)
    })
}


module.exports = model("User",User)