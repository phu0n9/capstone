const router = require('express').Router()
const Inventory = require('../model/inventory.model')
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

router.route('').get(jwtCheck,async (req,res) => {
    const page = req.query.page || 1// Page 
    const resPerPage = 5 // results per page
    await Inventory.find().sort({'createdAt':-1}).skip((page - resPerPage)).limit(resPerPage)
    .then(inventory =>{      
      res.send(inventory) // return data JSON   
    })
    .catch(err => console.log(err)) 
})

router.route('/:id').get(jwtCheck,async (req, res) =>{
  const itemId = req.params.id
  await Inventory.findById(itemId)
  .then((item) => res.send(item))
  .catch((err) => res.send(err))
})

router.route('/search').get(jwtCheck,async (req, res) =>{
  const keyword = req.query.keyword
  await Inventory.find({location:{"$regex":keyword,"$options":"i"}}).sort({'createdAt':-1})
    .then(inventory =>{      
      res.send(inventory) // return data JSON   
    })
    .catch(err => console.log(err))    
})


module.exports = router