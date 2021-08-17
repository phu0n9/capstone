const router = require('express').Router()
const Queue = require('../model/queue.model')
const Pusher = require('pusher')
const jwt = require('express-jwt')
const jwks = require('jwks-rsa')

const jwtCheck = jwt({
    secret: jwks.expressJwtSecret({
        cache: true,
        rateLimit: true,
        jwksRequestsPerMinute: 5,
        jwksUri: process.env.JWKS_URI
  }),
  audience: process.env.JWT_AUDIENCE,
  issuer: process.env.JWT_ISSUER,
  algorithms: ['RS256']
})

require('dotenv').config()

const pusher = new Pusher({
    appId: process.env.PUSHER_APP_ID,
    key: process.env.PUSHER_APP_KEY,
    secret: process.env.PUSHER_APP_SECRET,
    cluster: process.env.PUSHER_APP_CLUSTER,
    useTLS: true
})

router.route('').get(jwtCheck,async (req,res) => {
    await Queue.countDocuments(function(err,count){
        if(!err && count !== 0){
            Queue.find().sort({ 'createdAt': -1 })
            .exec((err, item) => {
                if (err) return (err)
                res.send(item)
            })
        }
    })
})

router.route('/delete/:id').delete(jwtCheck,async (req, res) => {
    await Queue.findByIdAndDelete(req.params.id).exec((err, inventory) => {
        if (err) return (err)
        const response = {
            message: "Successfully deleted",
            id: req.params.id
        }
        return res.status(200).send(response)
    })
})

router.route('/execute/:id').get(jwtCheck,async (req, res) => {
    await pusher.trigger('search','keyword',req.params.id)
    .then(() => res.status(200).send("sent request"))
    .catch((error)=>{console.log(error)})
})

module.exports = router